import numpy as np
from ur5_motion.ur5_fk import fk_ur5
from ur5_motion.ur5_ik import ur5_ik_analytic

def test_fk_matrix_shape_and_bottom_row():
    T = fk_ur5(np.zeros(6))
    assert T.shape == (4,4)
    assert np.allclose(T[3,:], [0,0,0,1])

def test_ik_then_fk_position_error_small():
    q = np.array([0.1,-1.2,1.4,0.1,1.2,-0.1])
    T = fk_ur5(q)
    sols = ur5_ik_analytic(T)
    assert len(sols) > 0
    q_best = min(sols, key=lambda s: np.linalg.norm(np.array(s)-q))
    T2 = fk_ur5(np.array(q_best))
    pos_err = np.linalg.norm(T[:3,3]-T2[:3,3])
    assert pos_err < 1e-3
