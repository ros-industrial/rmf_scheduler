# Copyright 2024-2025 ROS Industrial Consortium Asia Pacific
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse

import uvicorn

from .app import app

def main():
    """Execute the main logic of the command."""
    # Parse command line arguments
    parser = argparse.ArgumentParser()

    parser.add_argument('--host', type=str, default='localhost',
                        help='Server Host')
    parser.add_argument('--port', type=int, default=8000,
                        help='Server Port')
    parser.add_argument('-i', '--interactive',
                        action='store_true',
                        help='Interactive shell')
    parser.add_argument('--debug',
                        action='store_true',
                        help='Debug Mode')
    args = parser.parse_args()

    if args.debug:
        print('<Debug Mode>')

    uvicorn.run(
        app,
        host=args.host,
        port=args.port,
    )
