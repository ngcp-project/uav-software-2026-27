#!/usr/bin/env python3
"""Request a clean shutdown of LIA and all subscribers."""

from lia_client import LiaClient


if __name__ == "__main__":
    LiaClient("shutdown.py").shutdown()
