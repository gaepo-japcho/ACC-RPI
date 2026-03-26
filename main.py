#!/usr/bin/env python3
"""
ACC HMI Entry Point
Usage:
    python main.py          # Simulation mode (standalone)
    python main.py --can    # Real CAN mode (requires python-can + hardware)
"""
import sys

def main():
    from hmi_gui import main as gui_main
    gui_main()

if __name__ == "__main__":
    main()
