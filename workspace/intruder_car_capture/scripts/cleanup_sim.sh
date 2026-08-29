#!/usr/bin/env bash
# Kill sim processes safely. Pattern strings live in this FILE, not in the
# invoking shell's cmdline, so pkill -f cannot self-match the caller.
for p in roslaunch gzserver gzclient rosmaster; do
    if pkill -9 -f "[${p:0:1}]${p:1}" 2>/dev/null; then
        echo "killed: $p"
    fi
done
sleep 2
echo "--- remaining ---"
pgrep -a gzserver; pgrep -a gzclient; pgrep -a rosmaster; pgrep -af roslaunch
echo "cleanup done"
