# Python API Server Demo for the RMF2 Scheduler

This is a Sample Python Server for the RMF2 Scheduler.
It requires ``fastapi>=0.101.0``.

You can check the version using the following command

```bash
pip show fastapi
```

For **Ubuntu 22.04** users, the system installed FastAPI needs to be upgraded using ``pip``.

```bash
pip install -U fastapi
```

This is **NOT NEEDED** for **Ubuntu 24.04**.

## Usage Instructions

To start the API server, run the following command

```bash
rmf2_scheduler_server_py
```

Other options include.

```
  -h, --help         show this help message and exit
  --host HOST        Server Host
  --port PORT        Server Port
  --debug            Debug Mode
```
