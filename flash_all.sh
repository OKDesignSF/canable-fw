dfu-util -l \
  | grep -Eo 'path="[^"]+"' \
  | cut -d'"' -f2 \
  | while read -r path; do
      sudo dfu-util -p "$path" -d 0483:df11 -c 1 -i 0 -a 0 -s 0x08000000:leave -D build/canable-9fddea4-dirty.bin
    done
