#!/bin/bash
# ---------------------------------------------------------------
# Download hospital models for AWS RoboMaker world (Gazebo/Fuel)
# Portable version: works from any folder structure (no hardcoded paths)
# ---------------------------------------------------------------
set -e

# --- Detect paths dynamically ---
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"                       # one level up (e.g., hospital_sim)
MODELS_DIR="${SCRIPT_DIR}/fuel_models"

echo "Creating fuel_models directory at: ${MODELS_DIR}"
mkdir -p "$MODELS_DIR"
cd "$MODELS_DIR"

# --- Models used in hospital.world ---
MODELS=(
  CGMClassic StorageRack Chair Scrubs PatientWheelChair WhiteChipChair
  TrolleyBed SurgicalTrolley PotatoChipChair VisitorKidSit FemaleVisitorSit
  AdjTable MopCart3 MaleVisitorSit Drawer OfficeChairBlack ElderLadyPatient
  ElderMalePatient InstrumentCart1 InstrumentCart2 MetalCabinet BedTable
  BedsideTable AnesthesiaMachine TrolleyBedPatient Shower SurgicalTrolleyMed
  StorageRackCovered StorageRackCoverOpen KitchenSink Toilet ParkingTrolleyMin
  ParkingTrolleyMax PatientFSit MaleVisitorOnPhone FemaleVisitor
  MalePatientBed XRayMachine IVStand BloodPressureMonitor BMWCart BPCart VendingMachine
)

# --- If fuel_utility.py exists nearby, try using it ---
if [ -f "${ROOT_DIR}/fuel_utility.py" ]; then
    echo "Found fuel_utility.py, attempting automatic download..."
    cd "${ROOT_DIR}"
    python3 -m pip install -q requests beautifulsoup4
    if python3 fuel_utility.py aws-robomaker-hospital-world/worlds/hospital.world; then
        echo " fuel_utility.py succeeded"
        exit 0
    else
        echo " fuel_utility.py failed — using manual method."
        cd "$MODELS_DIR"
    fi
fi

# --- Manual download from Fuel ---
download_model() {
    local model_name=$1
    echo "Downloading $model_name..."

    # Try Fuel (Ignition + Gazebo) mirrors
    wget -q --show-progress -O "${model_name}.zip" \
        "https://fuel.gazebosim.org/1.0/OpenRobotics/models/${model_name}/tip/${model_name}.zip" \
        || wget -q --show-progress -O "${model_name}.zip" \
        "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/${model_name}/tip/${model_name}.zip" \
        || echo " Failed to download $model_name"

    if [ -f "${model_name}.zip" ]; then
        unzip -q -o "${model_name}.zip" -d "${MODELS_DIR}/${model_name}"
        rm -f "${model_name}.zip"
        echo "✓ $model_name downloaded"
    fi
}

# --- Download loop ---
for model in "${MODELS[@]}"; do
    if [ ! -d "$model" ]; then
        download_model "$model"
    else
        echo " $model already exists"
    fi
done

# --- Optional fallback: OSRF Gazebo models repo ---
echo ""
echo "Optionally clone the OSRF gazebo_models repository (~900MB)."
read -p "Clone now? (y/n): " -n 1 -r; echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    cd "$ROOT_DIR"
    if [ ! -d "gazebo_models" ]; then
        echo "Cloning Gazebo models repository..."
        git clone https://github.com/osrf/gazebo_models.git
    else
        echo "gazebo_models already exists."
    fi

    cd "$MODELS_DIR"
    for model in "${MODELS[@]}"; do
        if [ -d "${ROOT_DIR}/gazebo_models/${model}" ] && [ ! -e "${MODELS_DIR}/${model}" ]; then
            ln -s "${ROOT_DIR}/gazebo_models/${model}" "${MODELS_DIR}/${model}"
            echo " Linked $model from gazebo_models"
        fi
    done
fi

# --- Summary ---
echo ""
SUCCESS_COUNT=$(find "$MODELS_DIR" -mindepth 1 -maxdepth 1 -type d | wc -l)
echo "Successfully set up $SUCCESS_COUNT models in:"
echo "   $MODELS_DIR"
echo ""
echo "To use these models, ensure your GAZEBO_MODEL_PATH includes:"
echo "   $MODELS_DIR"
echo ""
echo "If some models are missing, you can re-run or use OSRF fallback."

