#!/usr/bin/env bash
# Script to show complete project structure and important file contents

set -e

PROJECT_ROOT="/home/taylor/workspaces/CleaningRobot/vision_production"
cd "$PROJECT_ROOT"

echo "================================================================================"
echo "DIRECTORY STRUCTURE (with all files)"
echo "================================================================================"
echo ""

# Show directory tree (exclude common ignore patterns)
if command -v tree &> /dev/null; then
    tree -a -I '__pycache__|*.pyc|.git|*.egg-info|build|install|log' --dirsfirst
else
    # Fallback: show both directories and files with indentation
    find . \( -type d -o -type f \) \
        ! -path '*/__pycache__*' \
        ! -path '*/.git*' \
        ! -path '*/build*' \
        ! -path '*/install*' \
        ! -path '*/log*' \
        ! -path '*/*.egg-info*' \
        | sort | while read -r path; do
            # Calculate depth and create indentation
            depth=$(echo "$path" | tr -cd '/' | wc -c)
            indent=$(printf '%*s' $((depth * 2)) '' | tr ' ' ' ')
            basename=$(basename "$path")
            if [ -d "$path" ]; then
                echo "${indent}${basename}/"
            else
                echo "${indent}${basename}"
            fi
        done
fi

echo ""
echo ""
echo "================================================================================"
echo "FILE CONTENTS (Code & Config Files)"
echo "================================================================================"
echo ""

# Function to print file with header
print_file() {
    local filepath="$1"
    local display_path="${filepath#./}"
    
    echo "--------------------------------------------------------------------------------"
    echo "FILE: $display_path"
    echo "--------------------------------------------------------------------------------"
    cat "$filepath"
    echo ""
    echo ""
}

# Function to print file size
print_size() {
    local filepath="$1"
    local display_path="${filepath#./}"
    local size=$(du -h "$filepath" | cut -f1)
    
    echo "[$display_path] - SIZE: $size"
}

# Find and process files
# Include: .py, .yaml, .yml, .xml, .cfg, .sh, .launch.py, Dockerfile, docker-compose.yml, .urdf, .xacro, .txt
# Exclude: .md, binaries (.onnx, .plan), __pycache__, build dirs

echo "=== Python Files ==="
echo ""
find . -type f -name "*.py" \
    ! -path '*/__pycache__*' \
    ! -path '*/build/*' \
    ! -path '*/install/*' \
    ! -path '*/log/*' \
    | sort | while read -r file; do
    print_file "$file"
done

echo "=== Launch Files ==="
echo ""
find . -type f -name "*.launch.py" \
    ! -path '*/build/*' \
    ! -path '*/install/*' \
    | sort | while read -r file; do
    print_file "$file"
done

echo "=== Configuration Files (YAML) ==="
echo ""
find . -type f \( -name "*.yaml" -o -name "*.yml" \) \
    ! -path '*/build/*' \
    ! -path '*/install/*' \
    | sort | while read -r file; do
    print_file "$file"
done

echo "=== Package XML Files ==="
echo ""
find . -type f -name "package.xml" \
    ! -path '*/build/*' \
    ! -path '*/install/*' \
    | sort | while read -r file; do
    print_file "$file"
done

echo "=== Setup Files ==="
echo ""
find . -type f \( -name "setup.py" -o -name "setup.cfg" \) \
    ! -path '*/build/*' \
    ! -path '*/install/*' \
    ! -path '*/*.egg-info/*' \
    | sort | while read -r file; do
    print_file "$file"
done

echo "=== CMake Files ==="
echo ""
find . -type f -name "CMakeLists.txt" \
    ! -path '*/build/*' \
    ! -path '*/install/*' \
    | sort | while read -r file; do
    print_file "$file"
done

echo "=== URDF/Xacro Files ==="
echo ""
find . -type f \( -name "*.urdf" -o -name "*.xacro" \) \
    ! -path '*/build/*' \
    ! -path '*/install/*' \
    | sort | while read -r file; do
    print_file "$file"
done

echo "=== Service Definition Files ==="
echo ""
find . -type f -name "*.srv" \
    ! -path '*/build/*' \
    ! -path '*/install/*' \
    | sort | while read -r file; do
    print_file "$file"
done

echo "=== Docker Files ==="
echo ""
find . -type f \( -name "Dockerfile" -o -name "docker-compose.yml" \) \
    | sort | while read -r file; do
    print_file "$file"
done

echo "=== Shell Scripts ==="
echo ""
find . -type f -name "*.sh" \
    ! -path '*/build/*' \
    ! -path '*/install/*' \
    | sort | while read -r file; do
    print_file "$file"
done

echo "=== Resource Files ==="
echo ""
find . -path "*/resource/*" -type f \
    ! -path '*/build/*' \
    ! -path '*/install/*' \
    | sort | while read -r file; do
    # These are usually empty marker files, just show they exist
    echo "RESOURCE: ${file#./}"
done
echo ""
echo ""

echo "================================================================================"
echo "BINARY/MODEL FILES (Size Only)"
echo "================================================================================"
echo ""

echo "=== Model Files ==="
find . -type f \( -name "*.onnx" -o -name "*.plan" -o -name "*.engine" \) \
    | sort | while read -r file; do
    print_size "$file"
done
echo ""

echo "=== Other Notable Files ==="
find . -type f \( -name "checksums.txt" -o -name "*.log" \) \
    ! -path '*/build/*' \
    ! -path '*/install/*' \
    | sort | while read -r file; do
    print_size "$file"
done
echo ""

echo "================================================================================"
echo "SUMMARY COMPLETE"
echo "================================================================================"
