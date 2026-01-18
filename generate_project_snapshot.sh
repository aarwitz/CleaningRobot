#!/bin/bash
# Generate a complete project snapshot with directory structure and all code files

OUTPUT_FILE="project_snapshot.txt"
PROJECT_DIR="/home/taylor/workspaces/CleaningRobot/vision_production"

cd "$PROJECT_DIR"

echo "Generating project snapshot..."
echo "=======================================" > "$OUTPUT_FILE"
echo "VISION PRODUCTION - PROJECT SNAPSHOT" >> "$OUTPUT_FILE"
echo "Generated: $(date)" >> "$OUTPUT_FILE"
echo "=======================================" >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

# Print directory structure
echo "=======================================" >> "$OUTPUT_FILE"
echo "DIRECTORY STRUCTURE" >> "$OUTPUT_FILE"
echo "=======================================" >> "$OUTPUT_FILE"
tree -I '__pycache__|*.pyc|*.onnx|*.plan|*.md|*.txt' -a >> "$OUTPUT_FILE" 2>/dev/null || find . -not -path '*/\.*' -not -path '*/__pycache__/*' -not -name '*.pyc' -not -name '*.onnx' -not -name '*.plan' -not -name '*.md' -not -name '*.txt' -print | sed 's|^\./||' | sort >> "$OUTPUT_FILE"
echo "" >> "$OUTPUT_FILE"

# Function to print file with header
print_file() {
    local filepath="$1"
    echo "" >> "$OUTPUT_FILE"
    echo "=======================================" >> "$OUTPUT_FILE"
    echo "FILE: $filepath" >> "$OUTPUT_FILE"
    echo "=======================================" >> "$OUTPUT_FILE"
    cat "$filepath" >> "$OUTPUT_FILE"
    echo "" >> "$OUTPUT_FILE"
}

# Find and print all code files
echo "=======================================" >> "$OUTPUT_FILE"
echo "FILE CONTENTS" >> "$OUTPUT_FILE"
echo "=======================================" >> "$OUTPUT_FILE"

# Python files
find . -type f -name "*.py" -not -path '*/__pycache__/*' -not -path '*/.*' | sort | while read -r file; do
    print_file "$file"
done

# YAML/YML config files
find . -type f \( -name "*.yaml" -o -name "*.yml" \) -not -path '*/.*' | sort | while read -r file; do
    print_file "$file"
done

# XML files (package.xml, CMakeLists.txt)
find . -type f \( -name "*.xml" -o -name "CMakeLists.txt" \) -not -path '*/.*' | sort | while read -r file; do
    print_file "$file"
done

# URDF/Xacro files
find . -type f \( -name "*.urdf" -o -name "*.xacro" \) -not -path '*/.*' | sort | while read -r file; do
    print_file "$file"
done

# Config files
find . -type f \( -name "*.cfg" -o -name "*.config" -o -name "*.conf" \) -not -path '*/.*' | sort | while read -r file; do
    print_file "$file"
done

# Docker files
if [ -f "docker/Dockerfile" ]; then
    print_file "docker/Dockerfile"
fi
if [ -f "docker/docker-compose.yml" ]; then
    print_file "docker/docker-compose.yml"
fi

# Shell scripts
find . -type f -name "*.sh" -not -path '*/.*' | sort | while read -r file; do
    print_file "$file"
done

# Service definition files
find . -type f -name "*.srv" -not -path '*/.*' | sort | while read -r file; do
    print_file "$file"
done

# Setup files
find . -type f \( -name "setup.py" -o -name "setup.cfg" \) -not -path '*/.*' | sort | while read -r file; do
    print_file "$file"
done

echo "=======================================" >> "$OUTPUT_FILE"
echo "END OF PROJECT SNAPSHOT" >> "$OUTPUT_FILE"
echo "=======================================" >> "$OUTPUT_FILE"

echo "Project snapshot generated: $OUTPUT_FILE"
echo "File size: $(du -h "$OUTPUT_FILE" | cut -f1)"
