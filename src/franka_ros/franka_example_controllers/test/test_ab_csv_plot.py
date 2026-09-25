#!/usr/bin/env python3
"""Common direct CSV / rostopic export parity and A/B/C plot checks; no ROS."""
import csv
import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root = Path(__file__).resolve().parents[4]
def module(name, path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result
plot=module('abplot',root/'plot_directional_cbf_real.py')
rec=module('abrec',Path(__file__).resolve().parents[1]/'scripts/record_directional_cbf.py')
class Tests(unittest.TestCase):
    def fixture(self, mode):
        rows=[]; seq=0
        for exp in (1,2,3,4):
            for k in range(101):
                t=k*.01;ns=100000000000+seq*10000000;seq+=1
                rows.append(NS(header=NS(stamp=NS(to_nsec=lambda ns=ns:ns),frame_id='fr3_link0'),
                    sample_index=seq,experiment_id=exp,dt=.01,cbf_h=.01,
                    cbf_constraint_nom=-.1,cbf_constraint_safe=.001,cbf_constraint_qp=.001,
                    energy_mode=mode,kinetic_energy_dir=.01*t,kinetic_energy_total=.02*t,
                    svd_jacobian=[1.]*6,Kmax=.02,alpha=float(exp),cbf_residual_tolerance=1e-5,
                    direction=[1.,0.,0.],cbf_active=exp!=1,task_aborted=False,
                    cbf_solution_applied=exp!=1,solver_status=int(exp!=1),
                    ee_position=[t,0.,0.],target_position=[1.,0.,0.],
                    ee_target_distance=1-t,ee_reference_distance=1-t,
                    q=[t]*7,dq=[1.]*7,tau_command=[t]*7,
                    u_nom=[t+j for j in range(7)],u_safe=[.5*t+j for j in range(7)]))
        return rows
    def export(self, folder, mode):
        rows=self.fixture(mode); direct=folder/('direct%d.csv'%mode); ros=folder/('ros%d.csv'%mode)
        converter=rec.CsvRows()
        with direct.open('w',newline='') as f:
            writer=csv.writer(f); extra=['q_%d'%j for j in range(1,8)]+['dq_%d'%j for j in range(1,8)]
            writer.writerow(rec.FIELDS+extra)
            for row in rows:writer.writerow(converter.convert(row)+row.q+row.dq)
        flat=[]
        for row in rows:
            out={'field.header.stamp':row.header.stamp.to_nsec(),'field.header.frame_id':'fr3_link0'}
            for key,value in vars(row).items():
                if key=='header':continue
                if isinstance(value,list):out.update({'field.'+key+str(i):v for i,v in enumerate(value)})
                else:out['field.'+key]=value
            flat.append(out)
        with ros.open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(flat[0]));writer.writeheader();writer.writerows(flat)
        return direct,ros
    def test_export_parity_and_modes(self):
        with tempfile.TemporaryDirectory() as temp:
            for mode in (0,1):
                direct,ros=self.export(Path(temp),mode)
                a,b=plot.load_csv(direct),plot.load_csv(ros)
                for key in ('kinetic_energy_total','kinetic_energy_dir','ee_target_distance',
                            'cbf_h','cbf_constraint_nom','cbf_constraint_safe','alpha','Kmax',
                            'q_1','dq_7','control_u_nom_3','control_u_safe_7'):
                    np.testing.assert_array_equal(a[key],b[key],err_msg=key)
                args=NS(off_id=1,on_ids=[2,3,4],segment=None,baseline_segment=None,
                        align='command',pre_motion=.2,duration=None)
                figures=plot.make_comparison(plt,plot.choose_trials(a,args))
                self.assertEqual(len(figures),4)
                self.assertEqual(len(figures[-1][2]),7)
                for ax in figures[-1][2]:self.assertEqual(len(ax.lines),7)
                energy_axes=figures[0][2]
                bounds=lambda ax:[line for line in ax.lines if line.get_label().startswith('Kmax')]
                self.assertEqual(len(bounds(energy_axes[1 if mode==0 else 0])),3)
                self.assertEqual(len(bounds(energy_axes[0 if mode==0 else 1])),0)
                plt.close('all')
    def test_external_common_baseline(self):
        with tempfile.TemporaryDirectory() as temp:
            a,_=self.export(Path(temp),0);b,_=self.export(Path(temp),1)
            args=NS(off_id=1,on_ids=[2,3,4],segment=None,baseline_segment=None,
                    align='command',pre_motion=.2,duration=None)
            trials=plot.choose_trials(plot.load_csv(a),args,plot.load_csv(b))
            self.assertEqual([t[1] for t in trials],[1,0,0,0])
            figs=plot.make_comparison(plt,trials)
            self.assertEqual(len(figs[2][2][0].lines),4) # A1,A2,A3 and zero; no B barrier
            plt.close('all')
if __name__=='__main__':unittest.main()
