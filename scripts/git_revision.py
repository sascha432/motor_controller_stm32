Import("env")

import subprocess


def get_git_revision():
    try:
        revision = subprocess.check_output(
            ["git", "rev-parse", "--short=8", "HEAD"],
            cwd=env.subst("$PROJECT_DIR"),
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "gunknown"

    return "g" + revision if revision else "gunknown"


env.Append(CPPDEFINES=[("GIT_REVISION", '\\"' + get_git_revision() + '\\"')])