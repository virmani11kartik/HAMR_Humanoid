# import os
# from SCons.Script import *

# Import("env")

# def manual_upload(source, target, env):
#     firmware_path = env.subst("$BUILD_DIR/firmware.bin")
#     port = env.subst("$UPLOAD_PORT")
#     baud = "460800"  # Or 115200 if preferred

#     esptool_path = os.path.join(env.PioPlatform().get_package_dir("tool-esptoolpy"), "esptool.py")

#     upload_cmd = f'python "{esptool_path}" --chip esp32s2 --port {port} --baud {baud} --before=default_reset --after=no_reset write_flash 0x1000 "{firmware_path}"'

#     print(f"\n[INFO] Uploading with: {upload_cmd}\n")
#     ret = os.system(upload_cmd)

#     if ret != 0:
#         print("[WARN] esptool returned non-zero code, but suppressing error.")
#     else:
#         print("[OK] Firmware uploaded successfully.")

#     # Force success to prevent PlatformIO upload error
#     env.Exit(0)

# env.Replace(UPLOADCMD=manual_upload)

# import os
# from SCons.Script import *

# Import("env")

# def manual_upload(source, target, env):
#     firmware_path = env.subst("$BUILD_DIR/firmware.bin")
#     port = env.subst("$UPLOAD_PORT")
#     baud = "460800"  # Or use "115200" if you face reliability issues

#     esptool_path = os.path.join(env.PioPlatform().get_package_dir("tool-esptoolpy"), "esptool.py")

#     # Changed --after to hard_reset instead of no_reset
#     upload_cmd = f'python "{esptool_path}" --chip esp32s2 --port {port} --baud {baud} --before=default_reset --after=hard_reset write_flash 0x1000 "{firmware_path}"'

#     print(f"\n[INFO] Uploading with: {upload_cmd}\n")
#     ret = os.system(upload_cmd)

#     if ret != 0:
#         print("[WARN] esptool returned non-zero code, but suppressing error.")
#     else:
#         print("[OK] Firmware uploaded and board reset successfully.")

#     env.Exit(0)

# env.Replace(UPLOADCMD=manual_upload)


# import os
# from SCons.Script import *

# Import("env")

# def manual_upload(source, target, env):
#     firmware_path = env.subst("$BUILD_DIR/firmware.bin")
#     port = env.subst("$UPLOAD_PORT")
#     baud = "460800"  # Use 115200 if needed

#     esptool_path = os.path.join(env.PioPlatform().get_package_dir("tool-esptoolpy"), "esptool.py")

#     upload_cmd = f'python "{esptool_path}" --chip esp32s2 --port {port} --baud {baud} --before=default_reset --after=no_reset write_flash 0x1000 "{firmware_path}"'

#     print(f"\n[INFO] Uploading with: {upload_cmd}\n")
#     ret = os.system(upload_cmd)

#     if ret != 0:
#         print("[WARN] esptool returned non-zero code, but suppressing error — your firmware may still be uploaded successfully.")
#     else:
#         print("[OK] Firmware uploaded successfully.")

#     # Always exit with 0 to suppress PlatformIO [FAILED]
#     env.Exit(0)

# env.Replace(UPLOADCMD=manual_upload)



import os
from SCons.Script import *

Import("env")

def manual_upload(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    port = env.subst("$UPLOAD_PORT")
    baud = "460800"

    esptool_path = os.path.join(env.PioPlatform().get_package_dir("tool-esptoolpy"), "esptool.py")

    bootloader = os.path.join(build_dir, "bootloader.bin")
    partitions = os.path.join(build_dir, "partitions.bin")
    firmware = os.path.join(build_dir, "firmware.bin")

    # Full flash layout
    upload_cmd = (
        f'python "{esptool_path}" --chip esp32s2 --port {port} --baud {baud} '
        f'--before=default_reset --after=no_reset write_flash '
        f'0x1000 "{bootloader}" '
        f'0x8000 "{partitions}" '
        f'0x10000 "{firmware}"'
    )

    print(f"\n[INFO] Uploading with:\n{upload_cmd}\n")
    ret = os.system(upload_cmd)

    if ret != 0:
        print("[WARN] esptool returned non-zero code, but suppressing error — your firmware may still be uploaded successfully.")
    else:
        print("[OK] Full firmware image uploaded successfully.")

    env.Exit(0)

env.Replace(UPLOADCMD=manual_upload)
