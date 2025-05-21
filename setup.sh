## Creating a virtual environment for the Detail Router project
python3 -m venv dr_env
source dr_env/bin/activate

## Installing required packages
pip install matplotlib rtree networkx
pip install --force-reinstall pkgs/LEFDEFParser-0.1-cp312-cp312-linux_x86_64.whl

## Running the Detail Router script
./run.sh > results.log