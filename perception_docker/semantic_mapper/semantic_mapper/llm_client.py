"""
llm_client.py — tiny HTTP wrapper around a local LLM.

Two backends, chosen by the LLM_BACKEND env var:

  LLM_BACKEND=ollama  (default)
      Talks to an Ollama server at LLM_BASE_URL (default
      http://localhost:11434). Model name in LLM_MODEL
      (default qwen2.5:3b-instruct). No API key needed.

  LLM_BACKEND=openai
      Talks to any OpenAI-compatible /chat/completions endpoint
      (OpenAI, Together, Groq, vLLM, llama-cpp-server, LM Studio, ...).
      Needs LLM_BASE_URL (e.g. https://api.openai.com/v1) and,
      if the server requires it, LLM_API_KEY.

The client keeps one `requests.Session` per instance, forces JSON
output when possible, and retries once on network errors. Callers get
a plain Python dict parsed from the model's JSON reply.

This module has no ROS dependencies on purpose — we want to be able
to unit-test it outside the ROS graph.
"""

from __future__ import annotations

import json
import os
import re
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import requests


# ─────────────────────────────────────────────────────────────────────
#  Config
# ─────────────────────────────────────────────────────────────────────
@dataclass
class LLMConfig:
    backend: str       = "ollama"
    base_url: str      = "http://localhost:11434"
    model: str         = "qwen2.5:3b-instruct"
    api_key: str       = ""
    temperature: float = 0.2
    timeout_s: float   = 30.0

    @classmethod
    def from_env(cls) -> "LLMConfig":
        def get(k, d): return os.environ.get(k, d)
        return cls(
            backend     = get("LLM_BACKEND",     "ollama").strip().lower(),
            base_url    = get("LLM_BASE_URL",    "http://localhost:11434").rstrip("/"),
            model       = get("LLM_MODEL",       "qwen2.5:3b-instruct"),
            api_key     = get("LLM_API_KEY",     ""),
            temperature = float(get("LLM_TEMPERATURE", "0.2")),
            timeout_s   = float(get("LLM_TIMEOUT_S", "30")),
        )


# ─────────────────────────────────────────────────────────────────────
#  Client
# ─────────────────────────────────────────────────────────────────────
class LLMClient:
    """
    Call as:
        llm = LLMClient.from_env()
        reply = llm.chat_json(system="...", user="...")
    `reply` is whatever dict the model wrote. If the reply wasn't valid
    JSON, .chat_json() raises ValueError with the raw text for logging.
    """

    def __init__(self, cfg: Optional[LLMConfig] = None):
        self.cfg = cfg or LLMConfig.from_env()
        self.sess = requests.Session()

    # ── Public ────────────────────────────────────────────────────
    @classmethod
    def from_env(cls) -> "LLMClient":
        return cls(LLMConfig.from_env())

    def chat_json(self, system: str, user: str,
                  temperature: Optional[float] = None) -> Dict[str, Any]:
        """Send system+user prompt, return parsed JSON dict."""
        text = self.chat_text(system, user, temperature)
        return _best_effort_json(text)

    def chat_text(self, system: str, user: str,
                  temperature: Optional[float] = None) -> str:
        """Send system+user prompt, return raw text."""
        t = self.cfg.temperature if temperature is None else float(temperature)
        if self.cfg.backend == "ollama":
            return self._ollama_chat(system, user, t)
        if self.cfg.backend in ("openai", "openai-compat"):
            return self._openai_chat(system, user, t)
        raise ValueError(f"Unknown LLM_BACKEND: {self.cfg.backend!r}")

    def ping(self) -> bool:
        """Return True if the server answers. Never raises."""
        try:
            if self.cfg.backend == "ollama":
                r = self.sess.get(f"{self.cfg.base_url}/api/tags",
                                  timeout=3.0)
                return r.status_code == 200
            r = self.sess.get(f"{self.cfg.base_url}/models",
                              headers=self._auth_header(), timeout=3.0)
            # Many servers return 401 on /models unauthenticated — that
            # still means "reachable".
            return r.status_code in (200, 401)
        except requests.RequestException:
            return False

    # ── Backends ──────────────────────────────────────────────────
    def _ollama_chat(self, system: str, user: str, temperature: float) -> str:
        url = f"{self.cfg.base_url}/api/chat"
        payload = {
            "model": self.cfg.model,
            "stream": False,
            "format": "json",             # ask Ollama to enforce JSON
            "options": {"temperature": temperature},
            "messages": [
                {"role": "system", "content": system},
                {"role": "user",   "content": user},
            ],
        }
        r = self._post_with_retry(url, payload)
        data = r.json()
        # Ollama's reply: {"message": {"role":"assistant","content":"..."}}
        try:
            return data["message"]["content"]
        except (KeyError, TypeError) as e:
            raise RuntimeError(f"unexpected Ollama reply: {data}") from e

    def _openai_chat(self, system: str, user: str, temperature: float) -> str:
        url = f"{self.cfg.base_url}/chat/completions"
        payload = {
            "model": self.cfg.model,
            "temperature": temperature,
            "response_format": {"type": "json_object"},
            "messages": [
                {"role": "system", "content": system},
                {"role": "user",   "content": user},
            ],
        }
        r = self._post_with_retry(url, payload, extra_headers=self._auth_header())
        data = r.json()
        try:
            return data["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as e:
            raise RuntimeError(f"unexpected OpenAI-compat reply: {data}") from e

    # ── HTTP plumbing ─────────────────────────────────────────────
    def _auth_header(self):
        if self.cfg.api_key:
            return {"Authorization": f"Bearer {self.cfg.api_key}"}
        return {}

    def _post_with_retry(self, url, payload, extra_headers=None, tries=2):
        headers = {"Content-Type": "application/json"}
        if extra_headers:
            headers.update(extra_headers)
        last_err = None
        for attempt in range(tries):
            try:
                r = self.sess.post(url, data=json.dumps(payload),
                                   headers=headers, timeout=self.cfg.timeout_s)
                r.raise_for_status()
                return r
            except requests.RequestException as e:
                last_err = e
                if attempt + 1 < tries:
                    time.sleep(0.4)
        raise RuntimeError(f"LLM request failed after {tries} tries: {last_err}")


# ─────────────────────────────────────────────────────────────────────
#  JSON rescue
# ─────────────────────────────────────────────────────────────────────
_JSON_OBJ = re.compile(r"\{.*\}", re.DOTALL)


def _best_effort_json(text: str) -> Dict[str, Any]:
    """
    Small models sometimes wrap their JSON in ```json ... ``` fences or
    prepend a one-line explanation. Strip the fence, pull out the first
    {...} block, and try json.loads.
    """
    t = text.strip()
    # ``` fences
    if t.startswith("```"):
        t = t.strip("`")
        # drop an optional 'json' tag on the first line
        nl = t.find("\n")
        if nl > 0:
            t = t[nl + 1:]
    # Try direct
    try:
        return json.loads(t)
    except json.JSONDecodeError:
        pass
    # Pull first { ... }
    m = _JSON_OBJ.search(t)
    if m:
        try:
            return json.loads(m.group(0))
        except json.JSONDecodeError:
            pass
    raise ValueError(f"LLM did not return valid JSON. Raw:\n{text}")