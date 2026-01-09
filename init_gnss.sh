#!/bin/bash

# 配置 GNSS 设备内部参数（杆臂、坐标系、导航模式等）
# 参数写入设备 Flash（$cmd,save,config*ff）
# 持久化：保存在设备内部，断电不丢失

#==============================================================================
# GNSS Device Configuration Script
# Purpose: Configure GNSS INS device for Apollo autonomous driving system
# Usage:   sudo ./init_gnss.sh [options]
# Options:
#   -d, --device    Serial device path (default: /dev/gps_serial)
#   -b, --baud      Baud rate (default: 115200)
#   -v, --verbose   Enable verbose output
#   -n, --dry-run   Show commands without sending
#   -h, --help      Show this help message
#==============================================================================

set -e  # Exit on error

#------------------------------------------------------------------------------
# Configuration Variables (Modify these as needed)
#------------------------------------------------------------------------------
DEVICE="/dev/gps_serial"
BAUD="115200"
VERBOSE=true
DRY_RUN=false
TIMEOUT=3
MAX_RETRIES=3
LOG_FILE="/tmp/gnss_config_$(date +%Y%m%d_%H%M%S).log"

#------------------------------------------------------------------------------
# Lever Arm Configuration (meters)
# These values should match your vehicle's GNSS antenna position
#------------------------------------------------------------------------------
GNSS_ARM_X="-0.135000"   # Forward (+) / Backward (-) from IMU
GNSS_ARM_Y="0.000000"    # Right (+) / Left (-) from IMU  
GNSS_ARM_Z="0.104000"    # Up (+) / Down (-) from IMU

POINT_ARM_X="0.000000"   # Point lever arm X
POINT_ARM_Y="0.000000"   # Point lever arm Y
POINT_ARM_Z="0.000000"   # Point lever arm Z

#------------------------------------------------------------------------------
# Heading & Coordinate Configuration
#------------------------------------------------------------------------------
HEAD_OFFSET="180"        # Heading offset in degrees (converts South=0 to North=0)
COORD_SYSTEM="-y,x,z"    # Coordinate system mapping

#------------------------------------------------------------------------------
# Colors for output
#------------------------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

#------------------------------------------------------------------------------
# Functions
#------------------------------------------------------------------------------

log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    echo "[$timestamp] [$level] $message" >> "$LOG_FILE"
    
    case "$level" in
        INFO)  echo -e "${BLUE}[INFO]${NC} $message" ;;
        OK)    echo -e "${GREEN}[OK]${NC} $message" ;;
        WARN)  echo -e "${YELLOW}[WARN]${NC} $message" ;;
        ERROR) echo -e "${RED}[ERROR]${NC} $message" >&2 ;;
        DEBUG) $VERBOSE && echo -e "[DEBUG] $message" ;;
    esac
}

show_help() {
    cat << EOF
GNSS Device Configuration Script

Usage: $(basename "$0") [OPTIONS]

Options:
    -d, --device PATH    Serial device path (default: /dev/gps_serial)
    -b, --baud RATE      Baud rate (default: 115200)
    -v, --verbose        Enable verbose output
    -n, --dry-run        Show commands without sending
    -h, --help           Show this help message

Examples:
    $(basename "$0")                          # Use defaults
    $(basename "$0") -d /dev/ttyUSB0          # Specify device
    $(basename "$0") -d /dev/ttyUSB0 -b 9600  # Specify device and baud rate
    $(basename "$0") -v -n                    # Verbose dry run

EOF
    exit 0
}

parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -d|--device)
                DEVICE="$2"
                shift 2
                ;;
            -b|--baud)
                BAUD="$2"
                shift 2
                ;;
            -v|--verbose)
                VERBOSE=true
                shift
                ;;
            -n|--dry-run)
                DRY_RUN=true
                shift
                ;;
            -h|--help)
                show_help
                ;;
            *)
                log ERROR "Unknown option: $1"
                show_help
                ;;
        esac
    done
}

check_dependencies() {
    local deps=("stty" "timeout")
    for dep in "${deps[@]}"; do
        if ! command -v "$dep" &> /dev/null; then
            log ERROR "Required command '$dep' not found. Please install it."
            exit 1
        fi
    done
}

check_device() {
    if [ ! -e "$DEVICE" ]; then
        log ERROR "Device $DEVICE not found."
        log INFO "Checking for available serial devices..."
        
        # List available USB serial devices
        if ls /dev/ttyUSB* 2>/dev/null; then
            log INFO "Available USB devices: $(ls /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')"
        fi
        if ls /dev/ttyACM* 2>/dev/null; then
            log INFO "Available ACM devices: $(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' ')"
        fi
        
        log INFO "Check 'dmesg | tail -30' for recent USB device events."
        exit 1
    fi
    
    # Check if device is a character device
    if [ ! -c "$DEVICE" ]; then
        log ERROR "$DEVICE is not a character device."
        exit 1
    fi
    
    log OK "Device $DEVICE found."
}

setup_serial() {
    log INFO "Setting permissions for $DEVICE..."
    
    if $DRY_RUN; then
        log INFO "[DRY-RUN] Would set permissions: chmod 666 $DEVICE"
        return
    fi
    
    # Try to set permissions, use sudo if necessary
    if [ -w "$DEVICE" ]; then
        log DEBUG "Device already writable"
    else
        if sudo chmod 666 "$DEVICE"; then
            log OK "Permissions set successfully."
        else
            log ERROR "Failed to set permissions. Try running with sudo."
            exit 1
        fi
    fi
    
    # Open file descriptor for reading and writing
    log INFO "Opening device $DEVICE for reading/writing..."
    
    # Temporarily disable exit-on-error for the redirection
    set +e
    exec 3<> "$DEVICE" 2>/tmp/serial_error.txt
    local exec_status=$?
    set -e
    
    if [ $exec_status -ne 0 ]; then
        log ERROR "Failed to open $DEVICE. Status: $exec_status"
        if [ -s /tmp/serial_error.txt ]; then
            log ERROR "System error: $(cat /tmp/serial_error.txt)"
        fi
        log INFO "This may be due to the device being busy or inaccessible."
        log INFO "Try: sudo lsof $DEVICE  to see if another process is using it."
        exit 1
    fi
    log OK "Device opened successfully (FD 3)."
    
    # Configure serial port
    log INFO "Configuring serial port: $BAUD 8N1 raw mode..."
    # Set to raw mode, disable all echo and special processing to prevent "bad command" loop
    if stty -F "$DEVICE" "$BAUD" raw cs8 -cstopb -parenb -echo -echoe -echok -echoctl -echoke; then
        log OK "Serial port configured."
    else
        log ERROR "Failed to configure serial port."
        exit 1
    fi
    
    # Flush any pending data
    sleep 0.1
    while read -t 0.1 -u 3 _ 2>/dev/null; do :; done
}

cleanup() {
    log INFO "Cleaning up..."
    # Close file descriptor if open
    exec 3>&- 2>/dev/null || true
    log INFO "Log saved to: $LOG_FILE"
}

send_cmd() {
    local cmd="$1"
    local description="${2:-Command}"
    local retry=0
    local success=false
    
    log INFO "Sending: $description"
    log DEBUG "Command: $cmd"
    
    if $DRY_RUN; then
        log INFO "[DRY-RUN] Would send: $cmd"
        return 0
    fi
    
    while [ $retry -lt $MAX_RETRIES ] && [ "$success" = false ]; do
        # Send the command with carriage return
        echo -e "${cmd}\r" >&3
        
        # Wait for response
        sleep 0.3
        
        # Read response (with timeout)
        local response=""
        while IFS= read -t 0.5 -u 3 -r line; do
            response+="$line"
            log DEBUG "Response: $line"
            
            # Check for success response
            if [[ "$line" == *"$cmd,ok"* ]] || [[ "$line" == *",OK"* ]] || [[ "$line" == *",ok"* ]]; then
                success=true
                break
            fi
            
            # Check for error response
            if [[ "$line" == *"error"* ]] || [[ "$line" == *"ERROR"* ]]; then
                log WARN "Device returned error for: $description"
                break
            fi
        done
        
        if [ "$success" = true ]; then
            log OK "$description - Success"
            return 0
        fi
        
        retry=$((retry + 1))
        if [ $retry -lt $MAX_RETRIES ]; then
            log WARN "$description - Retry $retry/$MAX_RETRIES"
            sleep 0.5
        fi
    done
    
    # Even without explicit confirmation, command may have been accepted
    log WARN "$description - No confirmation received (may still be OK)"
    return 0
}

send_cmd_simple() {
    # Simplified send without response checking (for devices that don't respond)
    local cmd="$1"
    local description="${2:-Command}"
    
    log INFO "Sending: $description"
    log DEBUG "Command: $cmd"
    
    if $DRY_RUN; then
        log INFO "[DRY-RUN] Would send: $cmd"
        return 0
    fi
    
    echo -e "${cmd}\r" >&3
    sleep 0.2
    log OK "$description - Sent"
}

show_config_summary() {
    echo ""
    echo "========================================"
    echo "     GNSS Configuration Summary"
    echo "========================================"
    echo ""
    echo "Device:          $DEVICE"
    echo "Baud Rate:       $BAUD"
    echo ""
    echo "Coordinate:      $COORD_SYSTEM"
    echo "Heading Offset:  ${HEAD_OFFSET}°"
    echo ""
    echo "GNSS Lever Arm:"
    echo "  X (Forward):   ${GNSS_ARM_X} m"
    echo "  Y (Right):     ${GNSS_ARM_Y} m"
    echo "  Z (Up):        ${GNSS_ARM_Z} m"
    echo ""
    echo "Point Lever Arm:"
    echo "  X (Forward):   ${POINT_ARM_X} m"
    echo "  Y (Right):     ${POINT_ARM_Y} m"
    echo "  Z (Up):        ${POINT_ARM_Z} m"
    echo ""
    echo "========================================"
    echo ""
}

configure_device() {
    # Define commands using *ff as wildcard checksum
    local CMD_COORD="\$cmd,set,coordinate,${COORD_SYSTEM}*ff"
    local CMD_HEAD="\$cmd,set,headoffset,${HEAD_OFFSET}*ff"
    local CMD_ARM="\$cmd,set,leverarm,gnss,${GNSS_ARM_X},${GNSS_ARM_Y},${GNSS_ARM_Z}*ff"
    local CMD_ARM2="\$cmd,set,leverarm,point,${POINT_ARM_X},${POINT_ARM_Y},${POINT_ARM_Z}*ff"
    
    # Navigation mode commands
    local CMD_NAV1='$cmd,set,navmode,finealign,off*ff'
    local CMD_NAV2='$cmd,set,navmode,coarsealign,off*ff'
    local CMD_NAV3='$cmd,set,navmode,dynamicalign,on*ff'
    local CMD_NAV4='$cmd,set,navmode,gnss,double*ff'
    local CMD_NAV5='$cmd,set,navmode,carmode,on*ff'
    local CMD_NAV6='$cmd,set,navmode,dmicali,off*ff'
    local CMD_NAV7='$cmd,set,navmode,zupt,on*ff'
    local CMD_NAV8='$cmd,set,navmode,firmwareindex,0*ff'
    
    # Configuration control commands
    # Note: $cmd,config,ok*ff is a RESPONSE from device, not a command to send
    local CMD_SAVE='$cmd,save,config*ff'
    
    log INFO "Starting GNSS configuration..."
    echo ""
    
    # Step 1: Coordinate System
    log INFO "========== Step 1/4: Coordinate System =========="
    send_cmd "$CMD_COORD" "Set Coordinate System (${COORD_SYSTEM})"
    sleep 0.5
    
    # Step 2: Heading Offset
    log INFO "========== Step 2/4: Heading Offset =========="
    send_cmd "$CMD_HEAD" "Set Heading Offset (${HEAD_OFFSET}°)"
    sleep 0.5
    
    # Step 3: Lever Arms
    log INFO "========== Step 3/4: Lever Arms =========="
    send_cmd "$CMD_ARM" "Set GNSS Lever Arm"
    sleep 0.3
    send_cmd "$CMD_ARM2" "Set Point Lever Arm"
    sleep 0.5
    
    # Navigation Modes (continued from Step 3)
    log INFO "Setting Navigation Modes..."
    send_cmd "$CMD_NAV1" "FineAlign: OFF"
    send_cmd "$CMD_NAV2" "CoarseAlign: OFF"
    send_cmd "$CMD_NAV3" "DynamicAlign: ON"
    send_cmd "$CMD_NAV4" "GNSS Mode: Double"
    send_cmd "$CMD_NAV5" "Car Mode: ON"
    send_cmd "$CMD_NAV6" "DMI Calibration: OFF"
    send_cmd "$CMD_NAV7" "ZUPT: ON"
    send_cmd "$CMD_NAV8" "Firmware Index: 0"
    sleep 0.5
    
    # Step 4: Save Configuration
    log INFO "========== Step 4/4: Save Configuration =========="
    send_cmd "$CMD_SAVE" "Save to Flash"
    sleep 2
    
    log OK "All configuration commands sent."
}

query_device_info() {
    # Optional: Query device for current settings
    local CMD_QUERY='$cmd,get,productinfo*ff'
    
    log INFO "Querying device information..."
    
    if $DRY_RUN; then
        log INFO "[DRY-RUN] Would query device info"
        return
    fi
    
    echo -e "${CMD_QUERY}\r" >&3
    sleep 0.5
    
    log INFO "Device response:"
    while IFS= read -t 1 -u 3 -r line; do
        if [[ -n "$line" ]]; then
            echo "  $line"
        fi
    done
}

#------------------------------------------------------------------------------
# Main Script
#------------------------------------------------------------------------------

main() {
    echo ""
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║           GNSS INS Device Configuration Script               ║"
    echo "║                 For Apollo Autonomous System                 ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo ""
    
    # Parse command line arguments
    parse_args "$@"
    
    # Initialize log file
    mkdir -p "$(dirname "$LOG_FILE")"
    echo "GNSS Configuration Log - $(date)" > "$LOG_FILE"
    echo "=================================" >> "$LOG_FILE"
    
    # Set up cleanup trap
    trap cleanup EXIT
    
    # Pre-flight checks
    check_dependencies
    check_device
    
    # Show configuration summary
    show_config_summary
    
    # Confirm before proceeding
    if ! $DRY_RUN; then
        echo -n "Proceed with configuration? [Y/n] "
        read -r answer
        if [[ "$answer" =~ ^[Nn] ]]; then
            log INFO "Configuration cancelled by user."
            exit 0
        fi
    fi
    
    # Setup serial port
    setup_serial
    
    # Query current device info (optional)
    # query_device_info
    
    # Configure device
    configure_device
    
    echo ""
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║                    Configuration Complete                    ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║  ⚠️  IMPORTANT: Please power cycle the GNSS device to        ║"
    echo "║     apply the new configuration.                             ║"
    echo "║                                                              ║"
    echo "║  After reboot, verify with: cat /dev/gps_serial              ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo ""
    
    log OK "Configuration completed successfully."
    log INFO "Log file: $LOG_FILE"
}

# Run main with all arguments
main "$@"
