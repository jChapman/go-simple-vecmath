# Go Simple Vecmath

A simple golang vector math library with a few quality of life features baked in.


### Installation

1. Fetch the package
```
go get github.com/amartyn1996/go-simple-vecmath
```

2. Import the package

```
import "github.com/amartyn1996/go-simple-vecmath"
```

### Usage


A vector can be created in a few ways

```
v1 := simple_vecmath.NewVector3(3,4,5)
v2 := simple_vecmath.NewVector3(5,4,3)

//If a function has 'NoOW' on the end, then a copy of the result is returned 
//and nothing is overwritten
v3 := v1.CrossNoOW(v2)

v4 := v3.Copy()

```

Almost all operations overwrite what they are called on

```
v1 := simple_vecmath.NewVector3(1,0,0)
v2 := simple_vecmath.NewVector3(0,1,0)
v1.Cross(v2) //v1 is now (0,0,1)
```

All operations support chaining

```
v1 := simple_vecmath.NewVector3(3,4,5)

//Construct an orthonormal basis from a few points
v2 := simple_vecmath.NewVector3(5,4,3).SubtractV(v1).Normalize()
v3 := simple_vecmath.NewVector3(2,0,0).SubtractV(v1)

v4 := v2.CrossNoOW(v3).Normalize()
v3.Cross2(v4,v2)
```

There are also matrix operations

```
v1 := simple_vecmath.NewVector3(3,4,5)
m1 := simple_vecmath.NewMatrix3Identity()

//Nothing happens (Identity Matrix)
v1.Matrix3Multiply(m1)

//Rotate 90 degrees counterclockwise around z and translate -2 x, 4 y, 3 z
v2 := simple_vecmath.NewVector3(0,-1,0)
v3 := simple_vecmath.NewVector3(1,0,0)
v4 := simple_vecmath.NewVector3(0,0,1)
v5 := simple_vecmath.NewVector3(-2,4,3)
m2 := simple_vecmath.NewMatrix4V(v2,v3,v4,v5,0,0,0,1)
v1.Matrix4Multiply(1.0, m2)

//Undo that operation
m2.Invert()
v1.Matrix4Multiply(1.0, m2)

```


