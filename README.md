# pure-python-collider
Collision detection in Python without Cython


Test whether a point falls within a polygon or ellipse.

This is a port of [garden.collider](https://github.com/kivy-garden/garden.collider).

Example usage:

```
>>> collider = Collide2DPoly([10., 10., 20., 30., 30., 10.],\
                             cache=True)
>>> (0.0, 0.0) in collider
False
>>> (20.0, 20.0) in collider
True
```