# Python API Server Demo for the RMF2 Scheduler

## Setup Instructions

Follow the [installation instructions](../../README.md) from source first.
Make sure the following are installed.
- `rmf2_scheduler`
- `rmf2_scheduler_py`

Setup `virtualenv` for Python

```bash
cd ../../../  # parent folder of repo root
mkdir rts-venv
python3 -m venv rts-venv
touch rts-venv/COLCON_IGNORE
. rts-venv/bin/activate
```

You might want to make sure that the venv is using up-to-date versions of the some foundational packages.

```bash
pip install -U pip setuptools build pyyaml typeguard jinja2
```

Install the package

```bash
cd rmf2_scheduler/demos/rmf2_scheduler_server/
pip install .
```

## Usage Instructions

To start the API server, run the following command

```bash
rmf2_scheduler_server
```

Other options include.

```
  -h, --help         show this help message and exit
  --host HOST        Server Host
  --port PORT        Server Port
  --debug            Debug Mode
```
