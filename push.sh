#!/bin/sh

# Use the installed local FTC toolchain when this script is part of the template.
local_deploy_dir=$(CDPATH= cd -- "$(dirname -- "$0")" 2>/dev/null && pwd -P) || exit 2
if [ -f "$local_deploy_dir/tools/deploy.py" ]; then
    if command -v python3 >/dev/null 2>&1; then
        exec python3 "$local_deploy_dir/tools/deploy.py" "$@"
    elif command -v python >/dev/null 2>&1; then
        exec python "$local_deploy_dir/tools/deploy.py" "$@"
    fi
    printf 'Python 3 is required to use the local FTC deployment tools.\n' >&2
    exit 2
fi

usage() {
    printf 'Usage: %s [--full] [robot-address]\n' "$0" >&2
}

fail_usage() {
    printf '%s\n' "$1" >&2
    usage
    exit 2
}

full_mode=0
robot_address=192.168.43.1
address_seen=0

for argument do
    case "$argument" in
        --full)
            if [ "$full_mode" -eq 1 ]; then
                fail_usage "--full may only be specified once."
            fi
            full_mode=1
            ;;
        -*)
            fail_usage "Unknown option: $argument"
            ;;
        *)
            if [ "$address_seen" -eq 1 ]; then
                fail_usage "Only one robot address may be specified."
            fi
            robot_address=$argument
            address_seen=1
            ;;
    esac
done

case "$robot_address" in
    *:*:*)
        fail_usage "Robot address must be a host or host:port."
        ;;
    *:*)
        robot_host=${robot_address%:*}
        robot_port=${robot_address##*:}
        ;;
    *)
        robot_host=$robot_address
        robot_port=5555
        ;;
esac

case "$robot_host" in
    ''|*[!A-Za-z0-9.-]*)
        fail_usage "Robot host contains invalid characters."
        ;;
esac

case "$robot_port" in
    [0-9]|[0-9][0-9]|[0-9][0-9][0-9]|[0-9][0-9][0-9][0-9]|[0-9][0-9][0-9][0-9][0-9])
        ;;
    *)
        fail_usage "Robot port must be an integer from 1 to 65535."
        ;;
esac

if [ "$robot_port" -lt 1 ] || [ "$robot_port" -gt 65535 ]; then
    fail_usage "Robot port must be an integer from 1 to 65535."
fi

script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" 2>/dev/null && pwd -P) || {
    printf 'Could not locate the deployment script directory.\n' >&2
    exit 2
}

target=$robot_host:$robot_port
if [ "$full_mode" -eq 1 ]; then
    gradle_task=installDebug
else
    gradle_task=deploySloth
fi

export ANDROID_SERIAL=$target
connected=0

cleanup() {
    deploy_status=$?
    trap - 0
    if [ "$connected" -eq 1 ]; then
        adb disconnect "$target" >/dev/null 2>&1 || :
    fi
    exit "$deploy_status"
}
trap cleanup 0

adb connect "$target"
connect_status=$?
if [ "$connect_status" -ne 0 ]; then
    exit "$connect_status"
fi
connected=1

device_state=$(adb -s "$target" get-state)
state_status=$?
if [ "$state_status" -ne 0 ]; then
    exit "$state_status"
fi
if [ "$device_state" != device ]; then
    printf 'ADB target %s is in state "%s", not "device".\n' "$target" "$device_state" >&2
    exit 1
fi

(cd "$script_dir" && ./gradlew "$gradle_task")
deploy_status=$?
exit "$deploy_status"
