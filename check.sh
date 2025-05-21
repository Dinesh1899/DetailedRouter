echo "DRC Checker for c17_out.def started........................................"
python3 sol/checker.py -i def/c17.def -o outputs/c17_out.def -l lef/sky130.lef
echo "DRC Checker for c17_out.def done..........................................."
sleep 2
echo "DRC Checker for add5_out.def started........................................"
python3 sol/checker.py -i def/add5.def -o outputs/add5_out.def -l lef/sky130.lef
echo "DRC Checker for add5_out.def done..........................................."
sleep 2
echo "DRC Checker for c432_out.def started........................................"
python3 sol/checker.py -i def/c432.def -o outputs/c432_out.def -l lef/sky130.lef
echo "DRC Checker for c432_out.def done..........................................."
sleep 2
echo "DRC Checker for c499_out.def started........................................"
python3 sol/checker.py -i def/c499.def -o outputs/c499_out.def -l lef/sky130.lef
echo "DRC Checker for c499_out.def done..........................................."
sleep 2
echo "DRC Checker for c6288_out.def started........................................"
python3 sol/checker.py -i def/c6288.def -o outputs/c6288_out.def -l lef/sky130.lef
echo "DRC Checker for c6288_out.def done..........................................."
sleep 2
echo "DRC Checker for c7552_out.def started........................................"
python3 sol/checker.py -i def/c7552.def -o outputs/c7552_out.def -l lef/sky130.lef
echo "DRC Checker for c7552_out.def done..........................................."
sleep 2
echo "DRC Checker for spm_out.def started........................................"
python3 sol/checker.py -i def/spm.def -o outputs/spm_out.def -l lef/sky130.lef
echo "DRC Checker for spm_out.def done..........................................."
