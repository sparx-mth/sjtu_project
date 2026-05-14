#!/bin/bash
# ============================================================
# fix_falcon_system_info.sh
#
# FALCON's printSystemInfo() (in exploration_utils/src/system_info.cpp)
# crashes the exploration_node at startup on Jetson (Tegra X1 / Xavier /
# AGX Orin). The function calls std::stol on the parsed output of
#   nvidia-smi --query-gpu=name,memory.total,memory.free
# without checking whether the call succeeded. On Jetson this nvidia-smi
# invocation typically returns an NVML error (or empty output), so
# std::stol receives a non-numeric string and throws
# std::invalid_argument, aborting the process before any voxels are ever
# published.
#
# This patch overwrites the file with a hardened version that:
#   * checks for empty strings before stol
#   * wraps stol calls in try/catch
#   * handles the case where nvidia-smi isn't installed or fails
# Functional behaviour is otherwise unchanged.
# ============================================================
set -e

TARGET=/catkin_ws/src/FALCON/falcon_planner/exploration_utils/src/system_info.cpp

if [ ! -f "$TARGET" ]; then
    echo "[fix_falcon_system_info] ERROR: $TARGET not found"
    echo "                          Was FALCON cloned at /catkin_ws/src/FALCON?"
    exit 1
fi

cat > "$TARGET" << 'CPP_EOF'
#include "system_info.h"

void printSystemInfo(std::string &output) {
  std::stringstream ss;
  ss << "|---------------------------------- System Info ----------------------------------|"
     << std::endl;
  std::string line;
  std::string cpu_name, cpu_cores, cpu_threads, cpu_freq;
  std::ifstream cpuinfo("/proc/cpuinfo");
  if (cpuinfo.is_open()) {
    while (getline(cpuinfo, line)) {
      if (line.find("model name") != std::string::npos) {
        cpu_name = line.substr(line.find(":") + 2);
      } else if (line.find("cpu cores") != std::string::npos) {
        cpu_cores = line.substr(line.find(":") + 2);
      } else if (line.find("siblings") != std::string::npos) {
        cpu_threads = line.substr(line.find(":") + 2);
      } else if (line.find("cpu MHz") != std::string::npos) {
        cpu_freq = line.substr(line.find(":") + 2);
      }
    }
    cpuinfo.close();
  }
  ss << "CPU Name: "      << (cpu_name.empty()    ? "N/A" : cpu_name)    << std::endl;
  ss << "CPU Cores: "     << (cpu_cores.empty()   ? "N/A" : cpu_cores)   << std::endl;
  ss << "CPU Threads: "   << (cpu_threads.empty() ? "N/A" : cpu_threads) << std::endl;
  ss << "CPU Frequency: " << (cpu_freq.empty()    ? "N/A" : cpu_freq + " MHz") << std::endl;

  std::ifstream meminfo("/proc/meminfo");
  std::string mem_total, mem_free;
  if (meminfo.is_open()) {
    while (getline(meminfo, line)) {
      if (line.find("MemTotal") != std::string::npos) {
        mem_total = line.substr(line.find(":") + 2);
      } else if (line.find("MemFree") != std::string::npos) {
        mem_free = line.substr(line.find(":") + 2);
      }
    }
    meminfo.close();
  }
  try {
    if (!mem_total.empty())
      ss << "Memory Total: " << std::stol(mem_total) / 1024.0 / 1024.0 << " GB" << std::endl;
    else
      ss << "Memory Total: N/A" << std::endl;
  } catch (const std::exception &) {
    ss << "Memory Total: N/A (parse failed)" << std::endl;
  }
  try {
    if (!mem_free.empty())
      ss << "Memory Free: "  << std::stol(mem_free)  / 1024.0 / 1024.0 << " GB" << std::endl;
    else
      ss << "Memory Free: N/A" << std::endl;
  } catch (const std::exception &) {
    ss << "Memory Free: N/A (parse failed)" << std::endl;
  }

  // GPU info — Jetson's nvidia-smi often doesn't support --query-gpu and
  // prints an NVML error instead. Treat any non-numeric value as N/A.
  std::string gpu_name = "N/A", gpu_mem_total, gpu_mem_free;
  std::string command =
      "nvidia-smi --query-gpu=name,memory.total,memory.free --format=csv,noheader 2>/dev/null";
  FILE *fp = popen(command.c_str(), "r");
  if (fp != NULL) {
    char buffer[1024];
    while (fgets(buffer, sizeof(buffer), fp) != NULL) {
      std::string line(buffer);
      size_t pos = line.find(",");
      if (pos == std::string::npos) continue;
      gpu_name = line.substr(0, pos);
      line = line.substr(pos + 1);
      pos = line.find(",");
      if (pos == std::string::npos) continue;
      gpu_mem_total = line.substr(0, pos);
      gpu_mem_free  = line.substr(pos + 1);
    }
    pclose(fp);
  }
  ss << "GPU Name: " << gpu_name << std::endl;
  try {
    if (!gpu_mem_total.empty())
      ss << "GPU Memory Total: " << std::stol(gpu_mem_total) / 1024.0 << " GB" << std::endl;
    else
      ss << "GPU Memory Total: N/A" << std::endl;
  } catch (const std::exception &) {
    ss << "GPU Memory Total: N/A (parse failed)" << std::endl;
  }
  try {
    if (!gpu_mem_free.empty())
      ss << "GPU Memory Free: "  << std::stol(gpu_mem_free)  / 1024.0 << " GB" << std::endl;
    else
      ss << "GPU Memory Free: N/A" << std::endl;
  } catch (const std::exception &) {
    ss << "GPU Memory Free: N/A (parse failed)" << std::endl;
  }

  ss << "|---------------------------------------------------------------------------------|"
     << std::endl;

  output = ss.str();
}
CPP_EOF

echo "[fix_falcon_system_info] patched $TARGET"
