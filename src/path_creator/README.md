# path_creator

# Libs
Conturs.py - lib for work with counturs

ex.:
```python
cnt_inst = Conturs(image)
cnts = cnt_inst.get_conturs(40)
```
image like:<br>
![N|Solid](../path_creator/test/img/mymap_22.jpg)

After get conturs:<br>
![N|Solid](../path_creator/img/img_1.png)

PathFinder.py - create path in the counturs

```python
pth = PathFinder(current_contur.contur, image.shape, 5, 1, start_point=start_point, debug_mode=True,neibor_distance=8)
covered_points = pth.get_route()
```

After get pathes:<br>
![N|Solid](../path_creator/img/img_2.png)

get_route is first version of path creation.


marker_lib.py - for markers in the map, like way points/line and goal indicator

# path_creator
path_creator_node.py - service<br>
subscribe /map topic<br>
call it:<br>
path_creator/get_by_id<br>
contur_creator/get_conturs<br>
contur_creator/get_by_id<br>
contur_creator/get_by_xy<br>
contur_creator/get_by_next_by_id<br>