"""Synthetic transform recovery, independent of ROS and saved hardware frames."""
import numpy as np
from fit_cad_cloud import register, rotation


def test_known_transform():
    rng = np.random.default_rng(2)
    model = rng.uniform([-7.5,-3.75,0], [7.5,3.75,20], (800,3))
    axis = np.arange(800)%3
    model[np.arange(800),axis] = np.where(np.arange(800)%2,1,-1)*np.array([7.5,3.75,10])[axis]
    model[:,2] += 10
    truth = np.array([270,-23,-5])
    observed = model @ rotation(np.deg2rad(37)).T + truth + rng.normal(0,.1,model.shape)
    _, result, distances = register(observed,model)[0]
    assert np.linalg.norm(result[:3]-truth) < .5
    assert np.median(distances) < .5

def test_tilted_transform():
    rng = np.random.default_rng(8)
    model = rng.uniform([-7,-4,0], [7,4,35], (800,3))
    truth = np.array([260,-20,-5])
    R = rotation(.6,.12,-.08)
    observed = model @ R.T + truth
    _, result, distances = register(observed,model,six_dof=True)[0]
    assert np.linalg.norm(result[:3]-truth)<.5
    assert np.median(distances)<.5

if __name__ == '__main__':
    test_known_transform()
    test_tilted_transform()
    print('Synthetic transform recovery passed')
