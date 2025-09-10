#!/usr/bin/env bash
set -euo pipefail

# init-arduino.sh
# Bootstrap an Arduino project repository

if [ $# -lt 1 ]; then
  echo "Usage: $0 <project-name>"
  exit 1
fi

NAME="$1"
mkdir -p "$NAME"/{src,include,lib,tests,docs}
cd "$NAME"

# Basic sketch file
cat > src/"$NAME".ino <<EOF
// $NAME.ino
// Entry point sketch

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}
EOF

# README
cat > README.md <<EOF
# $NAME

Arduino project initialized with \`init-arduino.sh\`.
EOF

# .gitignore
cat > .gitignore <<EOF
*.hex
*.elf
*.bin
*.o
*.d
build/
EOF

git init
git add .
git commit -m "Initial commit: Arduino project $NAME"
echo "Repository '$NAME' initialized."