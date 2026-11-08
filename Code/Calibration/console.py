"""Tiny terminal helpers for the interactive tools."""
import select
import sys
import termios
import tty


class Keys:
    """with Keys() as k: k.get(timeout) -> one key or None (cbreak mode, restored on exit)."""

    def __enter__(self):
        self.fd, self.old = sys.stdin.fileno(), termios.tcgetattr(sys.stdin)
        tty.setcbreak(self.fd)
        return self

    def get(self, timeout=0.2):
        return sys.stdin.read(1) if select.select([sys.stdin], [], [], timeout)[0] else None

    def __exit__(self, *a):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
