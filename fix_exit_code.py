# fix_exit_code.py
Import("env")

def after_upload(source, target, env):
    print("Upload complete. Manual reset required.")
    # Force success exit code (0)
    env.Exit(0)

env.AddPostAction("upload", after_upload)
