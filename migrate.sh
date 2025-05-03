#!/bin/bash

# Migration script to replace old structure with new modular structure

echo "Creating backup of current repository..."
BACKUP_DIR="/home/mdlw/pequi/humanoid/marta_control_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"
cp -r /home/mdlw/pequi/humanoid/marta_control/* "$BACKUP_DIR/"
echo "Backup created at: $BACKUP_DIR"

echo "Removing all current content (keeping only backup and new structure)..."
find /home/mdlw/pequi/humanoid/marta_control -mindepth 1 -not -path "/home/mdlw/pequi/humanoid/marta_control/new*" -not -path "/home/mdlw/pequi/humanoid/marta_control/migrate.sh" -delete

echo "Moving new structure to main directory..."
mv /home/mdlw/pequi/humanoid/marta_control/new/* /home/mdlw/pequi/humanoid/marta_control/
rmdir /home/mdlw/pequi/humanoid/marta_control/new

echo "Migration complete."
echo "You can now build the system with the setup.sh script:"
echo "cd /home/mdlw/pequi/humanoid/marta_control"
echo "./setup.sh"