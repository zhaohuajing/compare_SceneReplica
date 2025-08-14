#!/bin/bash

# Define output directory (change if needed)

# TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
Date=$(date +"%Y%m%d") #_%H%M%S")
OUTPUT_DIR="/root/compare_SceneReplica/package_logs/package_details"
mkdir -p "$OUTPUT_DIR"

echo "Saving package lists to $OUTPUT_DIR (date: $Date)..."

# 1. Save APT (Debian/Ubuntu) packages
if command -v dpkg > /dev/null; then
    echo "Saving APT package list..."
    dpkg-query -W -f='${Installed-Size}\t${Package}\n' | sort -n > "$OUTPUT_DIR/apt_packages.txt"
    ls -lt /var/lib/dpkg/info/ | grep '\.list$' > "$OUTPUT_DIR/apt_install_order.txt"
fi

# 2. Save Python pip packages
if command -v pip > /dev/null; then
    echo "Saving pip package list..."
    pip list > "$OUTPUT_DIR/pip_list.txt"
    pip list --format=freeze | xargs pip show | grep -E 'Name|Version|Location|Installer|Date' > "$OUTPUT_DIR/pip_metadata.txt"
fi

# 3. Save conda packages (if available) [output contact_graspnet_env only]
if command -v conda > /dev/null; then
    echo "Saving conda package list..."
    source /root/miniconda3/etc/profile.d/conda.sh
    conda activate contact_graspnet_env
    conda list > "$OUTPUT_DIR/conda_list.txt"
    conda list --revisions > "$OUTPUT_DIR/conda_revisions.txt"
fi

echo "Saving Package Logs Done."

#--------------------------

OUTPUT_DIR="/root/compare_SceneReplica/package_logs/package_lists"
mkdir -p "$OUTPUT_DIR"

# -------------------------
# 1. Save APT packages in simplified format
# -------------------------
echo "Saving APT packages..."

dpkg-query -W -f='${Package}==${Version}\n' | sort > "$OUTPUT_DIR/apt_requirements.txt"

# -------------------------
# 2. Save pip packages (global or in base)
# -------------------------
echo "Saving pip packages (base)..."

pip list --format=freeze | sort > "$OUTPUT_DIR/pip_requirements_base.txt"

# -------------------------
# 3. Save conda packages for contact_graspnet_env
# -------------------------
echo "Saving Conda environment..."

source /root/miniconda3/etc/profile.d/conda.sh
conda activate contact_graspnet_env

# Export environment as reproducible YAML
conda env export --no-builds > "$OUTPUT_DIR/contact_graspnet_env.yml"

# List conda packages in requirements-style
conda list --export | sort > "$OUTPUT_DIR/conda_requirements_contact_graspnet_env.txt"

# Also capture pip packages inside the conda environment
echo "Saving pip packages (inside contact_graspnet_env)..."
pip list --format=freeze | sort > "$OUTPUT_DIR/pip_requirements_contact_graspnet_env.txt"