python3 EE24M102.py
sleep 10
echo "Checking C17"
date
python3 checker.py -i ../def/c17.def -o ./c17_out.def -l ../lef/sky130.lef
date
sleep 2
echo "Checking ADD5"
date
python3 checker.py -i ../def/add5.def -o ./add5_out.def -l ../lef/sky130.lef
date
sleep 2
echo "Checking C432"
date
python3 checker.py -i ../def/c432.def -o ./c432_out.def -l ../lef/sky130.lef
date
sleep 4
echo "Checking C499"
date
python3 checker.py -i ../def/c499.def -o ./c499_out.def -l ../lef/sky130.lef
date
sleep 4
echo "Checking C6288"
date
python3 checker.py -i ../def/c6288.def -o ./c6288_out.def -l ../lef/sky130.lef
date
sleep 7
echo "Checking C7552"
date
python3 checker.py -i ../def/c7552.def -o ./c7552_out.def -l ../lef/sky130.lef
date
sleep 7
echo "Checking SPM"
date
python3 checker.py -i ../def/spm.def -o ./spm_out.def -l ../lef/sky130.lef
date