#!/usr/bin/env bash
# slopvr build: one compiler invocation per target, no build system.
#   ./build.sh            driver (release) -> build/slopvr/ (what SteamVR loads), build/libslopvr_vk.so + slopvr_vk.json, build/70-slopvr.rules
#   ./build.sh test       psvr2linuxtest (debug) -> build/psvr2linuxtest   (needs SDL2, wayland-client, libdrm)
#   ./build.sh all        both
#   ./build.sh install    driver, then register it with this user's SteamVR
#   ./build.sh register   registration only (SLOPVR_DIR selects the driver directory, default build/slopvr)
#   ./build.sh clean
# Modifiers: debug (driver with the -DSLOP_DEBUG diagnostics), gcc, clang, asan. CC overrides the compiler.
# The driver is checked against glibc 2.31 (the Steam Linux Runtime) after linking.
set -eu
cd "$(dirname "$0")"

# --- arguments ---------------------------------------------------------------
for arg in "$@"; do declare "$arg"='1'; done
if [[ "${driver:-0}${test:-0}${all:-0}${install:-0}${register:-0}${clean:-0}" == "000000" ]]; then driver='1'; fi
if [[ "${all:-0}" == "1" || "${install:-0}" == "1" ]]; then driver='1'; fi
if [[ "${all:-0}" == "1" ]]; then test='1'; fi
if [[ "${install:-0}" == "1" ]]; then register='1'; fi
if [[ "${clean:-0}" == "1" ]]; then rm -rf build; echo "[clean]"; fi

# --- compiler ----------------------------------------------------------------
if   [[ "${clang:-0}" == "1" ]]; then compiler="${CC:-clang}"
elif [[ "${gcc:-0}"   == "1" ]]; then compiler="${CC:-gcc}"
else compiler="${CC:-cc}"; fi
cc_common="-std=gnu11 -D_GNU_SOURCE -Wall -Wextra -Wno-unused-function -Wno-missing-field-initializers -Wno-unterminated-string-initialization -Wno-misleading-indentation -Wno-format-truncation -Wno-unknown-warning-option -Isrc"
if [[ "${asan:-0}" == "1" ]]; then cc_common="$cc_common -fsanitize=address"; fi
cc_release="-O2 -DNDEBUG $cc_common"
cc_debug="-O2 -g -DSLOP_DEBUG $cc_common"
cc_shared="-shared -fPIC -fvisibility=hidden -fno-plt -Wl,--no-undefined"

# --- udev rules (the driver and the test tool both need them): headset over usbfs, Sense controllers over hidraw
#     (USB bus 0003 / Bluetooth 0005, L 0E45 / R 0E46), accessible to the logged-in user -----------------------
if [[ "${driver:-0}${test:-0}" != "00" ]]; then
  mkdir -p build
  cat > build/70-slopvr.rules <<'EOF'
SUBSYSTEM=="usb", ATTR{idVendor}=="054c", ATTR{idProduct}=="0cde", TAG+="uaccess", MODE="0660"
KERNEL=="hidraw*", KERNELS=="000[35]:054C:0E4[56].*", TAG+="uaccess", MODE="0660"
EOF
fi

# --- driver: driver_slopvr.so + the Vulkan layer + the files around them ------
if [[ "${driver:-0}" == "1" ]]; then
  flags="$cc_release"; [[ "${debug:-0}" == "1" ]] && flags="$cc_debug"
  echo "[driver, $compiler]"
  mkdir -p build/slopvr/bin/linux64
  $compiler $flags $cc_shared -o build/slopvr/bin/linux64/driver_slopvr.so src/slopvr.c -lm -lpthread
  $compiler $flags $cc_shared -o build/libslopvr_vk.so src/vklayer.c -lpthread
  rm -rf build/slopvr/resources && cp -r resources build/slopvr/
  cat > build/slopvr/driver.vrdrivermanifest <<'EOF'
{
	"alwaysActivate": true,
	"name": "slopvr",
	"directory": "",
	"resourceOnly": false,
	"hmd_presence": [ "054C.0CDE" ]
}
EOF
  # implicit Vulkan layer: makes vrcompositor record the 120 Hz mode it selects (src/vklayer.c)
  printf '{"file_format_version":"1.2.0","layer":{"name":"VK_LAYER_slopvr_modes","type":"GLOBAL","library_path":"%s","api_version":"1.3.0","implementation_version":"1","description":"slopvr: sort DRM display modes by refresh rate","disable_environment":{"DISABLE_VK_LAYER_slopvr_modes":"1"}}}\n' \
    "$PWD/build/libslopvr_vk.so" > build/slopvr_vk.json
  # SteamVR runs inside the Steam Linux Runtime (glibc 2.31): nothing newer may be referenced (see src/glibc.h)
  if [[ -x "$(command -v objdump)" ]]; then
    for so in build/slopvr/bin/linux64/driver_slopvr.so build/libslopvr_vk.so; do
      bad=$(objdump -T "$so" | grep -oE 'GLIBC_2\.[0-9]+' | sort -uV | awk -F. '$2 > 31')
      if [[ -n "$bad" ]]; then echo "$so references glibc symbols newer than 2.31 (${bad//$'\n'/ }); add them to src/glibc.h:"; objdump -T "$so" | grep -E "GLIBC_2\.(3[2-9]|[4-9][0-9])"; exit 1; fi
    done
    echo "[glibc ok: $(objdump -T build/slopvr/bin/linux64/driver_slopvr.so | grep -oE 'GLIBC_2\.[0-9]+' | sort -uV | tail -1) max]"
  fi
fi

# --- test tool: camera view, tracking debugger, calibration flow --------------
if [[ "${test:-0}" == "1" ]]; then
  echo "[psvr2linuxtest, $compiler]"
  if [[ -x "$(command -v pkg-config)" ]] && pkg-config --exists sdl2 wayland-client libdrm 2>/dev/null; then
    libs="$(pkg-config --cflags --libs sdl2 wayland-client libdrm)"
  else
    libs="-I/usr/include/SDL2 -I/usr/include/libdrm -lSDL2 -lwayland-client -ldrm"
  fi
  # shellcheck disable=SC2086
  $compiler $cc_debug -o build/psvr2linuxtest src/psvr2linuxtest.c -lm -lpthread $libs
fi

# --- register: tell this user's SteamVR about the driver ----------------------
# What vrpathreg adddriver does (vrpathreg itself does not run outside steam-run on NixOS): the driver directory goes into
# ~/.config/openvr/openvrpaths.vrpath, older slopvr entries are dropped; the Vulkan layer json goes to the user's implicit
# layer directory, where vrcompositor picks it up like Steam's own overlay layer. Idempotent, run after every rebuild.
# The displayPortTrainingMode setting the panel needs is written by the driver itself at start.
if [[ "${register:-0}" == "1" ]]; then
  steamvr=${STEAMVR:-$HOME/.local/share/Steam/steamapps/common/SteamVR}
  dir=$(readlink -f "${SLOPVR_DIR:-$PWD/build/slopvr}")
  if [[ ! -f "$dir/driver.vrdrivermanifest" ]]; then echo "driver directory not found: $dir (build first, or set SLOPVR_DIR)"; exit 1; fi
  if [[ ! -d "$steamvr/bin/linux64" ]]; then echo "SteamVR is not installed at $steamvr (set STEAMVR); nothing registered"; exit 0; fi
  if [[ ! -x "$(command -v jq)" ]]; then echo "jq is needed to edit openvrpaths.vrpath"; exit 1; fi
  paths=$HOME/.config/openvr/openvrpaths.vrpath
  mkdir -p "$(dirname "$paths")"
  [[ -s "$paths" ]] || jq -n --arg s "$steamvr" --arg c "$HOME/.local/share/Steam/config" --arg l "$HOME/.local/share/Steam/logs" \
    '{config:[$c],external_drivers:null,jsonid:"vrpathreg",log:[$l],runtime:[$s],version:1}' > "$paths"
  keep=()
  while read -r d; do
    [[ -n "$d" ]] || continue
    if [[ "$d" == "$dir" || ! -d "$d" ]] || grep -qs '"name" *: *"slopvr"' "$d/driver.vrdrivermanifest"; then continue; fi
    keep+=("$d")
  done < <(jq -r '.external_drivers // [] | .[]' "$paths")
  keep+=("$dir")
  tmp=$(mktemp)
  jq --argjson arr "$(printf '%s\n' "${keep[@]}" | jq -R . | jq -s .)" '.external_drivers = $arr' "$paths" > "$tmp" && mv "$tmp" "$paths"
  layer="$dir/../share/vulkan/implicit_layer.d/slopvr_vk.json"; [[ -f "$layer" ]] || layer="$PWD/build/slopvr_vk.json"
  if [[ -f "$layer" ]]; then mkdir -p "$HOME/.local/share/vulkan/implicit_layer.d"; install -m 644 "$layer" "$HOME/.local/share/vulkan/implicit_layer.d/slopvr_vk.json"; fi
  echo "registered $dir"
  if pgrep -x vrserver >/dev/null 2>&1; then echo "SteamVR is running: it loads the new driver after a restart"; fi
  rules=$(readlink -f "$dir/../lib/udev/rules.d/70-slopvr.rules" 2>/dev/null || true); [[ -f "$rules" ]] || rules="$PWD/build/70-slopvr.rules"
  if ! grep -rqs '054c", ATTR{idProduct}=="0cde"' /etc/udev/rules.d /run/udev/rules.d /run/current-system/sw/lib/udev/rules.d; then
    cat <<EOF
udev rules are not installed (needed to open the headset and the controllers):
  NixOS:        services.slopvr.enable = true;  (see README)
  until reboot: pkexec sh -c 'mkdir -p /run/udev/rules.d && cp $rules /run/udev/rules.d/ && udevadm control --reload && udevadm trigger'
EOF
  fi
fi
