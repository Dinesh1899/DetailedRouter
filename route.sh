echo "Deleting previous output def files"
rm -rf outputs/*

sleep 5
echo "Routing c17.def started........................................"
python3 sol/router.py -i def/c17.def -g gr/c17.guide -l lef/sky130.lef -o outputs/c17_out.def
echo "Routing c17.def Done..........................................."
sleep 5
echo "Routing add5.def started........................................"
python3 sol/router.py -i def/add5.def -g gr/add5.guide -l lef/sky130.lef -o outputs/add5_out.def
echo "Routing add5.def done..........................................."
sleep 5
echo "Routing c432.def started........................................"
python3 sol/router.py -i def/c432.def -g gr/c432.guide -l lef/sky130.lef -o outputs/c432_out.def
echo "Routing c432.def done..........................................."
sleep 5
echo "Routing c499.def started........................................"
python3 sol/router.py -i def/c499.def -g gr/c499.guide -l lef/sky130.lef -o outputs/c499_out.def
echo "Routing c499.def done..........................................."
sleep 5
echo "Routing c6288.def started........................................"
python3 sol/router.py -i def/c6288.def -g gr/c6288.guide -l lef/sky130.lef -o outputs/c6288_out.def
echo "Routing c6288.def done..........................................."
sleep 5
echo "Routing c7552.def started........................................"
python3 sol/router.py -i def/c7552.def -g gr/c7552.guide -l lef/sky130.lef -o outputs/c7552_out.def
echo "Routing c7552.def done..........................................."
sleep 5
echo "Routing spm.def started........................................"
python3 sol/router.py -i def/spm.def -g gr/spm.guide -l lef/sky130.lef -o outputs/spm_out.def
echo "Routing spm.def done..........................................."
