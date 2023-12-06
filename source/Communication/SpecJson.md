# Request

One line, `\n` to delimit end of request. Must be a single json object.

Field "index" will be copied as is to response

Field "query" is used for order type

# Query alive

Responds in field "response" true

# Query config

Responds in field "response" "mode" mode accepted or unknown mode

## Contains field mode

If field mode is "Millimeter2D" (default) : 

Coords are centered on the bottom left of the table, in millimeter and degrees.

If field mode is "Float2D" :

Coords are centered on the center of the table, in meter and radians

If field mode is "Float3D" :

Coords are centered on center of the table, given as a 4x4 matrix, in meter

# Query data

Returns data gathered from the external cameras

An array is expected in field "filter".

If filter contains "all", then all data from the cameras is returned

## 3D data

Returned as an array of objects under field "3D data", with (if mode is 2D):
- type
- x
- y
- r

type being in filter

## 2D data

Array of objects under field "2D data", with :
- name (camera name)
- width (width in pixel of image)
- height (height in pixels of image)
- xfov (horizontal field of view, degrees)
- yfov (vertical field of view, degrees)

### If filter contains Aruco

field "arucos" contains array of objects with :
- index (number of the aruco tag)
- array of corners, with x y (in image space, pixels)

### If filter contains Yolo

field "yolo" contains array of objects with :
- index (number of the yolo class)
- tlx, tly (top left) (in image space, pixels)
- brx, bry (bottom right)

