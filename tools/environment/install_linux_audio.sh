#!/usr/bin/env bash
# Local Ubuntu amd64 fallback when sudo is unavailable. No system/user settings change.
set -e
companionbot_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
test -x "$companionbot_root/.venv/bin/python"
test "$(dpkg --print-architecture)" = amd64
companionbot_debs="$(mktemp -d /tmp/companionbot-audio-XXXXXX)"
cd "$companionbot_debs"
apt-get download libportaudio2 libasound2-plugins espeak-ng libespeak-ng1 espeak-ng-data \
    libpcaudio0 libsonic0 pulseaudio-utils
mkdir -p "$companionbot_root/.venv/native"
for companionbot_deb in ./*.deb; do
    dpkg-deb -x "$companionbot_deb" "$companionbot_root/.venv/native"
done
ln -sf libportaudio.so.2 "$companionbot_root/.venv/native/usr/lib/x86_64-linux-gnu/libportaudio.so"
echo "Local audio runtime ready. Source scripts/env.sh; original debs: $companionbot_debs"
