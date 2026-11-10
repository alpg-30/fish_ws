"""Python 2.7 / 3.x compatibility helpers for fish_tools."""

from __future__ import absolute_import

import errno
import io
import math
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time

PY2 = sys.version_info[0] == 2

try:
    text_type = unicode
except NameError:
    text_type = str

try:
    import queue
except ImportError:  # Python 2
    import Queue as queue

try:
    monotonic = time.monotonic
except AttributeError:  # Python 2
    monotonic = time.time


class CommandTimeout(Exception):
    pass


class TemporaryDirectory(object):
    """TemporaryDirectory replacement for Python 2.7."""

    def __enter__(self):
        self.name = tempfile.mkdtemp()
        return self.name

    def __exit__(self, exc_type, exc_value, traceback):
        shutil.rmtree(self.name, ignore_errors=True)
        return False


def open_text(path, mode="r"):
    return io.open(path, mode, encoding="utf-8")


def makedirs(path):
    try:
        os.makedirs(path)
    except OSError as exc:
        if exc.errno != errno.EEXIST or not os.path.isdir(path):
            raise


def atomic_replace(source, destination):
    # POSIX rename is atomic and replaces the destination.
    os.rename(source, destination)


def disk_free_mb(path):
    stats = os.statvfs(path)
    return (stats.f_bavail * stats.f_frsize) / (1024.0 * 1024.0)


def posix_path(path):
    return path.replace(os.sep, "/")


def isfinite(value):
    return not (math.isnan(value) or math.isinf(value))


def popen_session_kwargs():
    if PY2:
        return {"preexec_fn": os.setsid}
    return {"start_new_session": True}


def devnull():
    return open(os.devnull, "rb")


def wait_process(process, timeout_s):
    deadline = monotonic() + float(timeout_s)
    while monotonic() < deadline:
        if process.poll() is not None:
            return True
        time.sleep(0.05)
    return process.poll() is not None


def run_command(argv, timeout_s):
    """Run argv and return (returncode, stdout_text, stderr_text)."""
    null = devnull()
    try:
        process = subprocess.Popen(
            argv,
            stdin=null,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            **popen_session_kwargs()
        )
    finally:
        null.close()

    result = {}

    def _communicate():
        stdout, stderr = process.communicate()
        result["stdout"] = stdout
        result["stderr"] = stderr

    reader = threading.Thread(target=_communicate)
    reader.daemon = True
    reader.start()
    reader.join(float(timeout_s))
    if reader.is_alive():
        try:
            process.kill()
        except OSError:
            pass
        reader.join(2.0)
        raise CommandTimeout(
            "command timed out after {0}s".format(timeout_s))

    stdout = result.get("stdout") or b""
    stderr = result.get("stderr") or b""
    if not isinstance(stdout, str):
        stdout = stdout.decode("utf-8", "replace")
    if not isinstance(stderr, str):
        stderr = stderr.decode("utf-8", "replace")
    return process.returncode, stdout, stderr
