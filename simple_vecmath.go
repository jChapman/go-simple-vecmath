package main

import (
	"fmt"
	"math"
)

const ePSILON float64 = 0.00000000001

func EpsilonEquals(number1, number2, epsilon float64) bool {
	return math.Abs(number1-number2) < epsilon
}

// Vector3  -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

type Vector3 struct {
	X, Y, Z float64
}

func NewVector3(X, Y, Z float64) *Vector3 {
	return &Vector3{X, Y, Z}
}

func (v *Vector3) ToString() string {
	return fmt.Sprintf("(%g,%g,%g)", v.X, v.Y, v.Z)
}

func (v *Vector3) Copy() *Vector3 {
	return NewVector3(v.X, v.Y, v.Z)
}

func (v *Vector3) Set(X, Y, Z float64) *Vector3 {
	v.X, v.Y, v.Z = X, Y, Z
	return v
}

func (v *Vector3) SetV(vector *Vector3) *Vector3 {
	v.X, v.Y, v.Z = vector.X, vector.Y, vector.Z
	return v
}

func (v *Vector3) Add(X, Y, Z float64) *Vector3 {
	v.X += X
	v.Y += Y
	v.Z += Z
	return v
}

func (v *Vector3) AddV(vector *Vector3) *Vector3 {
	v.X += vector.X
	v.Y += vector.Y
	v.Z += vector.Z
	return v
}

func (v *Vector3) Subtract(X, Y, Z float64) *Vector3 {
	v.X -= X
	v.Y -= Y
	v.Z -= Z
	return v
}

func (v *Vector3) SubtractV(vector *Vector3) *Vector3 {
	v.X -= vector.X
	v.Y -= vector.Y
	v.Z -= vector.Z
	return v
}

func (v *Vector3) SubtractVNoOW(vector *Vector3) *Vector3 {
	return NewVector3(0,0,0).SubtractV2(v,vector)
}

func (v *Vector3) SubtractV2(vector1, vector2 *Vector3) *Vector3 {
	v.X = vector1.X - vector2.X
	v.Y = vector1.Y - vector2.Y
	v.Z = vector1.Z - vector2.Z
	return v
}

func (v *Vector3) Scale(scalar float64) *Vector3 {
	v.X *= scalar
	v.Y *= scalar
	v.Z *= scalar
	return v
}

func (v *Vector3) ScaleV(vector *Vector3) *Vector3 {
	v.X *= vector.X
	v.Y *= vector.Y
	v.Z *= vector.Z
	return v
}

func (v *Vector3) Zero() *Vector3 {
	v.X, v.Y, v.Z = 0, 0, 0
	return v
}

func (v *Vector3) Length() float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

func (v *Vector3) Length2() float64 {
	return v.X*v.X + v.Y*v.Y + v.Z*v.Z
}

func (v *Vector3) Normalize() *Vector3 {
	length := v.Length()
	v.X /= length
	v.Y /= length
	v.Z /= length
	return v
}

func (v *Vector3) Dot(vector *Vector3) float64 {
	return v.X*vector.X + v.Y*vector.Y + v.Z*vector.Z
}

func (v *Vector3) CrossNoOW(vector *Vector3) *Vector3 {
	return NewVector3(0,0,0).Cross2(v,vector)
}

func (v *Vector3) Cross(vector *Vector3) *Vector3 {
	crossx := v.Y*vector.Z - v.Z*vector.Y
	crossy := v.Z*vector.X - v.X*vector.Z
	crossz := v.X*vector.Y - v.Y*vector.X
	return v.Set(crossx, crossy, crossz)
}

func (v *Vector3) Cross2(vector1 *Vector3, vector2 *Vector3) *Vector3 {	
	crossx := vector1.Y*vector2.Z - vector1.Z*vector2.Y
	crossy := vector1.Z*vector2.X - vector1.X*vector2.Z
	crossz := vector1.X*vector2.Y - vector1.Y*vector2.X
	return v.Set(crossx, crossy, crossz)
}

func (v *Vector3) Lerp(alpha float64, vector1 *Vector3, vector2 *Vector3) *Vector3 {
	alphaFrac := 1 - alpha
	v.X = alpha * vector1.X + alphaFrac * vector2.X;
	v.Y = alpha * vector1.Y + alphaFrac * vector2.Y;
	v.Z = alpha * vector1.Z + alphaFrac * vector2.Z;
	return v
}

func (v *Vector3) BarycentricCoordinates( a, b, c, p *Vector3 ) *Vector3 {
	AB := b.SubtractVNoOW(a)
	AC := c.SubtractVNoOW(a)
	BC := c.SubtractVNoOW(b)
	
	normal := AB.CrossNoOW( AC )
	normalLenSq := normal.Dot(normal)

	XP := p.SubtractVNoOW( b )
	XN := BC.CrossNoOW( XP )
	alpha := XN.Dot(normal) / normalLenSq

	XP.SubtractV2(p, a)
	XN.Cross2(XP, AC)
	beta := XN.Dot(normal) / normalLenSq

	gamma := 1 - alpha - beta

	return v.Set(alpha, beta, gamma)
}

func (v *Vector3) BarycentricCoordinatesNoOW( a, b, c *Vector3 ) *Vector3 {
	return NewVector3(0,0,0).BarycentricCoordinates(a,b,c,v)
}

func (v *Vector3) Clamp(min, max float64) *Vector3 {
	v.X = math.Max(math.Min(v.X, max), min)
	v.Y = math.Max(math.Min(v.Y, max), min)
	v.Z = math.Max(math.Min(v.Z, max), min)
	return v
}

func (v *Vector3) EpsilonEquals(vector *Vector3, epsilon float64) bool {
	differencex := math.Abs(v.X - vector.X)
	differencey := math.Abs(v.Y - vector.Y)
	differencez := math.Abs(v.Z - vector.Z)

	maxDifference := math.Max(math.Max(differencex, differencey), differencez)
	return maxDifference < epsilon
}

//Rodrigues' rotation formula
func (v *Vector3) Rotate(axis *Vector3, angle float64) *Vector3 {
	axisNorm := axis.Copy().Normalize()
	cosAngle := math.Cos(angle)
	term1 := v.Copy().Scale(cosAngle)
	term2 := axisNorm.Copy().Cross(v).Scale(math.Sin(angle))
	term3 := axisNorm.Copy().Scale(v.Dot(axisNorm) * (1 - cosAngle))

	v.SetV(term1.AddV(term2).AddV(term3))
	return v
}

func (v *Vector3) Negate() *Vector3 {
	v.X, v.Y, v.Z = -v.X, -v.Y, -v.Z
	return v
}

func (v *Vector3) ScaleAdd(scalar float64, vector *Vector3) *Vector3 {
	v.X += scalar * vector.X
	v.Y += scalar * vector.Y
	v.Z += scalar * vector.Z
	return v
}

func (v *Vector3) Swap(vector *Vector3) *Vector3 {
	tmp := v.X
	v.X = vector.X
	vector.X = tmp
	tmp = v.Y
	v.Y = vector.Y
	vector.Y = tmp
	tmp = v.Z
	v.Z = vector.Z
	vector.Z = tmp
	return v
}

func (v *Vector3) Matrix3Multiply(m *Matrix3) *Vector3 {
	newX := v.X*m.XX + v.Y*m.XY + v.Z*m.XZ
	newY := v.X*m.YX + v.Y*m.YY + v.Z*m.YZ
	newZ := v.X*m.ZX + v.Y*m.ZY + v.Z*m.ZZ
	v.X, v.Y, v.Z = newX, newY, newZ
	return v
}

func (v *Vector3) Matrix4Multiply(w float64, m *Matrix4) *Vector3 {	
	newX := v.X*m.XX + v.Y*m.XY + v.Z*m.XZ + w*m.XW
	newY := v.X*m.YX + v.Y*m.YY + v.Z*m.YZ + w*m.YW
	newZ := v.X*m.ZX + v.Y*m.ZY + v.Z*m.ZZ + w*m.ZW
	v.X, v.Y, v.Z = newX, newY, newZ
	return v
}

// Matrix3  -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

type Matrix3 struct {
	XX, XY, XZ float64
	YX, YY, YZ float64
	ZX, ZY, ZZ float64
}

func NewMatrix3Identity() *Matrix3 {
	m := Matrix3{1, 0, 0,
		0, 1, 0,
		0, 0, 1}
	return &m
}

func NewMatrix3(XX, XY, XZ, YX, YY, YZ, ZX, ZY, ZZ float64) *Matrix3 {
	m := Matrix3{XX, XY, XZ,
		YX, YY, YZ,
		ZX, ZY, ZZ}
	return &m
}

func NewMatrix3V(vx, vy, vz *Vector3) *Matrix3 {
	m := Matrix3{vx.X, vy.X, vz.X,
		vx.Y, vy.Y, vz.Y,
		vx.Z, vy.Z, vz.Z}
	return &m
}

func (m *Matrix3) ToString() string {
	return fmt.Sprintf("( %g,%g,%g | %g,%g,%g | %g,%g,%g )",
		m.XX, m.XY, m.XZ,
		m.YX, m.YY, m.YZ,
		m.ZX, m.ZY, m.ZZ)
}

func (m *Matrix3) Copy() *Matrix3 {
	return &Matrix3{
		m.XX, m.XY, m.XZ,
		m.YX, m.YY, m.YZ,
		m.ZX, m.ZY, m.ZZ}
}

func (m *Matrix3) Identity() *Matrix3 {
	m.XX, m.XY, m.XZ = 1, 0, 0
	m.YX, m.YY, m.YZ = 0, 1, 0
	m.ZX, m.ZY, m.ZZ = 0, 0, 1
	return m
}

func (m *Matrix3) Set(XX, XY, XZ, YX, YY, YZ, ZX, ZY, ZZ float64) *Matrix3 {
	m.XX, m.XY, m.XZ = XX, XY, XZ
	m.YX, m.YY, m.YZ = YX, YY, YZ
	m.ZX, m.ZY, m.ZZ = ZX, ZY, ZZ
	return m
}

func (m *Matrix3) SetV(vx, vy, vz *Vector3) *Matrix3 {
	m.XX, m.XY, m.XZ = vx.X, vy.X, vz.X
	m.YX, m.YY, m.YZ = vx.Y, vy.Y, vz.Y
	m.ZX, m.ZY, m.ZZ = vx.Z, vy.Z, vz.Z
	return m
}

func (m *Matrix3) SetM(matrix *Matrix3) *Matrix3 {
	m.XX, m.XY, m.XZ = matrix.XX, matrix.XY, matrix.XZ
	m.YX, m.YY, m.YZ = matrix.YX, matrix.YY, matrix.YZ
	m.ZX, m.ZY, m.ZZ = matrix.ZX, matrix.ZY, matrix.ZZ
	return m
}

func (m *Matrix3) Determinant() float64 {
	diagonal1 := m.XX*m.YY*m.ZZ + m.XY*m.YZ*m.ZX + m.XZ*m.YX*m.ZY
	diagonal2 := -m.ZX*m.YY*m.XZ - m.ZY*m.YZ*m.XX - m.ZZ*m.YX*m.XY
	return diagonal1 + diagonal2
}

func (m *Matrix3) Invert() *Matrix3 {

	determinant := m.Determinant()

	if EpsilonEquals(determinant, 0, ePSILON) {
		panic("Matrix does not have an inverse")
	}

	iDet := 1 / determinant

	nxx := (m.YY*m.ZZ - m.YZ*m.ZY) * iDet
	nyx := -(m.YX*m.ZZ - m.YZ*m.ZX) * iDet
	nzx := (m.YX*m.ZY - m.YY*m.ZX) * iDet
	nxy := -(m.XY*m.ZZ - m.XZ*m.ZY) * iDet
	nyy := (m.XX*m.ZZ - m.XZ*m.ZX) * iDet
	nzy := -(m.XX*m.ZY - m.XY*m.ZX) * iDet
	nxz := (m.XY*m.YZ - m.XZ*m.YY) * iDet
	nyz := -(m.XX*m.YZ - m.XZ*m.YX) * iDet
	nzz := (m.XX*m.YY - m.XY*m.YX) * iDet

	m.XX, m.XY, m.XZ = nxx, nxy, nxz
	m.YX, m.YY, m.YZ = nyx, nyy, nyz
	m.ZX, m.ZY, m.ZZ = nzx, nzy, nzz

	return m
}

func (m *Matrix3) Multiply(m3 *Matrix3) *Matrix3 {
	nxx := m.XX*m3.XX + m.XY*m3.YX + m.XZ*m3.ZX
	nxy := m.XX*m3.XY + m.XY*m3.YY + m.XZ*m3.ZY
	nxz := m.XX*m3.XZ + m.XY*m3.YZ + m.XZ*m3.ZZ
	nyx := m.YX*m3.XX + m.YY*m3.YX + m.YZ*m3.ZX
	nyy := m.YX*m3.XY + m.YY*m3.YY + m.YZ*m3.ZY
	nyz := m.YX*m3.XZ + m.YY*m3.YZ + m.YZ*m3.ZZ
	nzx := m.ZX*m3.XX + m.ZY*m3.YX + m.ZZ*m3.ZX
	nzy := m.ZX*m3.XY + m.ZY*m3.YY + m.ZZ*m3.ZY
	nzz := m.ZX*m3.XZ + m.ZY*m3.YZ + m.ZZ*m3.ZZ

	m.XX, m.XY, m.XZ = nxx, nxy, nxz
	m.YX, m.YY, m.YZ = nyx, nyy, nyz
	m.ZX, m.ZY, m.ZZ = nzx, nzy, nzz

	return m
}

func (m *Matrix3) MultiplyV(vector *Vector3) *Matrix3 {
	m.XX *= vector.X; m.XY *= vector.Y; m.XZ *= vector.Z
	m.YX *= vector.X; m.YY *= vector.Y; m.YZ *= vector.Z
	m.ZX *= vector.X; m.ZY *= vector.Y; m.ZZ *= vector.Z
	return m
}

func (m *Matrix3) Scale(scalar float64) *Matrix3 {
	m.XX *= scalar; m.XY *= scalar; m.XZ *= scalar
	m.YX *= scalar; m.YY *= scalar; m.YZ *= scalar
	m.ZX *= scalar; m.ZY *= scalar; m.ZZ *= scalar
	return m
}

func (m *Matrix3) Add(m3 *Matrix3) *Matrix3 {	
	m.XX += m3.XX; m.XY += m3.XY; m.XZ += m3.XZ
	m.YX += m3.YX; m.YY += m3.YY; m.YZ += m3.YZ
	m.ZX += m3.ZX; m.ZY += m3.ZY; m.ZZ += m3.ZZ
	return m
}

func (m *Matrix3) Subtract(m3 *Matrix3) *Matrix3 {	
	m.XX -= m3.XX; m.XY -= m3.XY; m.XZ -= m3.XZ
	m.YX -= m3.YX; m.YY -= m3.YY; m.YZ -= m3.YZ
	m.ZX -= m3.ZX; m.ZY -= m3.ZY; m.ZZ -= m3.ZZ
	return m
}

func (m *Matrix3) IsInvertable() bool {
	return !EpsilonEquals(m.Determinant(), 0, ePSILON)
}

func (m *Matrix3) Swap(matrix *Matrix3) *Matrix3 {
	tmp := m.XX; m.XX = matrix.XX; matrix.XX = tmp
	tmp = m.XY; m.XY = matrix.XY; matrix.XY = tmp
	tmp = m.XZ; m.XZ = matrix.XZ; matrix.XZ = tmp
	tmp = m.YX; m.YX = matrix.YX; matrix.YX = tmp
	tmp = m.YY; m.YY = matrix.YY; matrix.YY = tmp
	tmp = m.YZ; m.YZ = matrix.YZ; matrix.YZ = tmp
	tmp = m.ZX; m.ZX = matrix.ZX; matrix.ZX = tmp
	tmp = m.ZY; m.ZY = matrix.ZY; matrix.ZY = tmp
	tmp = m.ZZ; m.ZZ = matrix.ZZ; matrix.ZZ = tmp
	return m
}

func (m *Matrix3) Transpose() *Matrix3 {
	tmp := m.XY; m.XY = m.YX; m.YX = tmp
	tmp = m.XZ; m.XZ = m.ZX; m.ZX = tmp
	tmp = m.YZ; m.YZ = m.ZY; m.ZY = tmp
	return m
}

// Matrix4  -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

type Matrix4 struct {
	XX, XY, XZ, XW float64
	YX, YY, YZ, YW float64
	ZX, ZY, ZZ, ZW float64
	WX, WY, WZ, WW float64
}

func NewMatrix4Identity() *Matrix4 {
	m := Matrix4{
    		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1}
	return &m
}

func NewMatrix4(XX, XY, XZ, XW, YX, YY, YZ, YW, ZX, ZY, ZZ, ZW, WX, WY, WZ, WW float64) *Matrix4 {
	m := Matrix4{
    		XX, XY, XZ, XW,
		YX, YY, YZ, YW,
		ZX, ZY, ZZ, ZW,
		WX, WY, WZ, WW}
	return &m
}

func NewMatrix4V(vx, vy, vz, vw *Vector3, WX, WY, WZ, WW float64) *Matrix4 {
	m := Matrix4{
    		vx.X, vy.X, vz.X, vw.X,
		vx.Y, vy.Y, vz.Y, vw.Y,
		vx.Z, vy.Z, vz.Z, vw.Z,
		WX  , WY  , WZ  , WW  }
	return &m
}

func (m *Matrix4) ToString() string {
	return fmt.Sprintf("( %g,%g,%g,%g | %g,%g,%g,%g | %g,%g,%g,%g | %g,%g,%g,%g )",
		m.XX, m.XY, m.XZ, m.XW,
		m.YX, m.YY, m.YZ, m.YW,
		m.ZX, m.ZY, m.ZZ, m.ZW,
		m.WX, m.WY, m.WZ, m.WW)
}

func (m *Matrix4) Copy() *Matrix4 {
	return &Matrix4{
		m.XX, m.XY, m.XZ, m.XW,
		m.YX, m.YY, m.YZ, m.YW,
		m.ZX, m.ZY, m.ZZ, m.ZW,
		m.WX, m.WY, m.WZ, m.WW}
}

func (m *Matrix4) Identity() *Matrix4 {
	m.XX, m.XY, m.XZ, m.XW = 1, 0, 0, 0
	m.YX, m.YY, m.YZ, m.YW = 0, 1, 0, 0
	m.ZX, m.ZY, m.ZZ, m.ZW = 0, 0, 1, 0
	m.WX, m.WY, m.WZ, m.WW = 0, 0, 0, 1
	return m
}

func (m *Matrix4) Set(XX, XY, XZ, XW, YX, YY, YZ, YW, ZX, ZY, ZZ, ZW, WX, WY, WZ, WW float64) *Matrix4 {
	m.XX, m.XY, m.XZ, m.XW = XX, XY, XZ, XW
	m.YX, m.YY, m.YZ, m.YW = YX, YY, YZ, YW
	m.ZX, m.ZY, m.ZZ, m.ZW = ZX, ZY, ZZ, ZW
	m.WX, m.WY, m.WZ, m.WW = WX, WY, WZ, WW
	return m
}

func (m *Matrix4) SetV(vx, vy, vz, vw *Vector3, WX, WY, WZ, WW float64) *Matrix4 {
	m.XX, m.XY, m.XZ, m.XW = vx.X, vy.X, vz.X, vw.X
	m.YX, m.YY, m.YZ, m.YW = vx.Y, vy.Y, vz.Y, vw.Y
	m.ZX, m.ZY, m.ZZ, m.ZW = vx.Z, vy.Z, vz.Z, vw.Z
	m.WX, m.WY, m.WZ, m.WW = WX  , WY  , WZ  , WW
	return m
}

func (m *Matrix4) SetM(matrix *Matrix4) *Matrix4 {
	m.XX, m.XY, m.XZ, m.XW = matrix.XX, matrix.XY, matrix.XZ, matrix.XW
	m.YX, m.YY, m.YZ, m.YW = matrix.YX, matrix.YY, matrix.YZ, matrix.YW
	m.ZX, m.ZY, m.ZZ, m.ZW = matrix.ZX, matrix.ZY, matrix.ZZ, matrix.ZW
	m.WX, m.WY, m.WZ, m.WW = matrix.WX, matrix.WY, matrix.WZ, matrix.WW
	return m
}

func (m *Matrix4) Determinant() float64 {

	submatrix := NewMatrix3(m.YY,m.YZ,m.YW,m.ZY,m.ZZ,m.ZW,m.WY,m.WZ,m.WW)
	det1 := submatrix.Determinant()

	submatrix.Set(m.YX,m.YZ,m.YW,m.ZX,m.ZZ,m.ZW,m.WX,m.WZ,m.WW)
	det2 := submatrix.Determinant()

	submatrix.Set(m.YX,m.YY,m.YW,m.ZX,m.ZY,m.ZW,m.WX,m.WY,m.WW)
	det3 := submatrix.Determinant()

	submatrix.Set(m.YX,m.YY,m.YZ,m.ZX,m.ZY,m.ZZ,m.WX,m.WY,m.WZ)
	det4 := submatrix.Determinant()

	return m.XX * det1 - m.XY * det2 + m.XZ * det3 - m.XW * det4
}

func (m *Matrix4) MultiplyV(vector *Vector3, w float64) *Matrix4 {
	m.XX *= vector.X; m.XY *= vector.Y; m.XZ *= vector.Z; m.XW *= w
	m.YX *= vector.X; m.YY *= vector.Y; m.YZ *= vector.Z; m.YW *= w
	m.ZX *= vector.X; m.ZY *= vector.Y; m.ZZ *= vector.Z; m.ZW *= w
	m.WX *= vector.X; m.WY *= vector.Y; m.WZ *= vector.Z; m.WW *= w
	return m
}

func (m *Matrix4) Multiply(m4 *Matrix4) *Matrix4 {
	nxx := m.XX*m4.XX + m.XY*m4.YX + m.XZ*m4.ZX + m.XW*m4.WX
	nxy := m.XX*m4.XY + m.XY*m4.YY + m.XZ*m4.ZY + m.XW*m4.WY
	nxz := m.XX*m4.XZ + m.XY*m4.YZ + m.XZ*m4.ZZ + m.XW*m4.WZ
	nxw := m.XX*m4.XW + m.XY*m4.YW + m.XZ*m4.ZW + m.XW*m4.WW
	
	nyx := m.YX*m4.XX + m.YY*m4.YX + m.YZ*m4.ZX + m.YW*m4.WX
	nyy := m.YX*m4.XY + m.YY*m4.YY + m.YZ*m4.ZY + m.YW*m4.WY
	nyz := m.YX*m4.XZ + m.YY*m4.YZ + m.YZ*m4.ZZ + m.YW*m4.WZ
	nyw := m.YX*m4.XW + m.YY*m4.YW + m.YZ*m4.ZW + m.YW*m4.WW

	nzx := m.ZX*m4.XX + m.ZY*m4.YX + m.ZZ*m4.ZX + m.ZW*m4.WX
	nzy := m.ZX*m4.XY + m.ZY*m4.YY + m.ZZ*m4.ZY + m.ZW*m4.WY
	nzz := m.ZX*m4.XZ + m.ZY*m4.YZ + m.ZZ*m4.ZZ + m.ZW*m4.WZ
	nzw := m.ZX*m4.XW + m.ZY*m4.YW + m.ZZ*m4.ZW + m.ZW*m4.WW

	nwx := m.WX*m4.XX + m.WY*m4.YX + m.WZ*m4.ZX + m.WW*m4.WX
	nwy := m.WX*m4.XY + m.WY*m4.YY + m.WZ*m4.ZY + m.WW*m4.WY
	nwz := m.WX*m4.XZ + m.WY*m4.YZ + m.WZ*m4.ZZ + m.WW*m4.WZ
	nww := m.WX*m4.XW + m.WY*m4.YW + m.WZ*m4.ZW + m.WW*m4.WW

	m.XX, m.XY, m.XZ, m.XW = nxx, nxy, nxz, nxw
	m.YX, m.YY, m.YZ, m.YW = nyx, nyy, nyz, nyw
	m.ZX, m.ZY, m.ZZ, m.ZW = nzx, nzy, nzz, nzw
	m.WX, m.WY, m.WZ, m.WW = nwx, nwy, nwz, nww

	return m
}

func (m *Matrix4) Scale(scalar float64) *Matrix4 {
	m.XX *= scalar; m.XY *= scalar; m.XZ *= scalar; m.XW *= scalar
	m.YX *= scalar; m.YY *= scalar; m.YZ *= scalar; m.YW *= scalar
	m.ZX *= scalar; m.ZY *= scalar; m.ZZ *= scalar; m.ZW *= scalar
	m.WX *= scalar; m.WY *= scalar; m.WZ *= scalar; m.WW *= scalar
	return m
}

func (m *Matrix4) Add(m4 *Matrix4) *Matrix4 {	
	m.XX += m4.XX; m.XY += m4.XY; m.XZ += m4.XZ; m.XW += m4.XW
	m.YX += m4.YX; m.YY += m4.YY; m.YZ += m4.YZ; m.YW += m4.YW
	m.ZX += m4.ZX; m.ZY += m4.ZY; m.ZZ += m4.ZZ; m.ZW += m4.ZW
	m.WX += m4.WX; m.WY += m4.WY; m.WZ += m4.WZ; m.WW += m4.WW
	return m
}

func (m *Matrix4) Subtract(m4 *Matrix4) *Matrix4 {	
	m.XX -= m4.XX; m.XY -= m4.XY; m.XZ -= m4.XZ; m.XW -= m4.XW
	m.YX -= m4.YX; m.YY -= m4.YY; m.YZ -= m4.YZ; m.YW -= m4.YW
	m.ZX -= m4.ZX; m.ZY -= m4.ZY; m.ZZ -= m4.ZZ; m.ZW -= m4.ZW
	m.WX -= m4.WX; m.WY -= m4.WY; m.WZ -= m4.WZ; m.WW -= m4.WW
	return m
}

func (m *Matrix4) IsInvertable() bool {
	return !EpsilonEquals(m.Determinant(), 0, ePSILON)
}

func (m *Matrix4) Transpose() *Matrix4 {
	tmp := m.XY; m.XY = m.YX; m.YX = tmp
	tmp = m.XZ; m.XZ = m.ZX; m.ZX = tmp
	tmp = m.XW; m.XW = m.WX; m.WX = tmp
	tmp = m.YZ; m.YZ = m.ZY; m.ZY = tmp
	tmp = m.YW; m.YW = m.WY; m.WY = tmp
	tmp = m.ZW; m.ZW = m.WZ; m.WZ = tmp
	return m
}

func (m *Matrix4) Invert() *Matrix4 {

	determinant := m.Determinant()

	if EpsilonEquals(determinant, 0, ePSILON) {
		panic("Matrix does not have an inverse")
	}

	XX := m.XX; XY := m.XY; XZ := m.XZ; XW := m.XW
	YX := m.YX; YY := m.YY; YZ := m.YZ; YW := m.YW
	ZX := m.ZX; ZY := m.ZY; ZZ := m.ZZ; ZW := m.ZW
	WX := m.WX; WY := m.WY; WZ := m.WZ; WW := m.WW

	//TODO: hardcode the determinant math so I can get rid of this temporary matrix. Sooo much typing... 
	tmp:= NewMatrix3(0,0,0,0,0,0,0,0,0) 
	
	m.XX = tmp.Set(YY,YZ,YW,ZY,ZZ,ZW,WY,WZ,WW).Determinant()
	m.XY = -tmp.Set(YX,YZ,YW,ZX,ZZ,ZW,WX,WZ,WW).Determinant()
	m.XZ = tmp.Set(YX,YY,YW,ZX,ZY,ZW,WX,WY,WW).Determinant()
	m.XW = -tmp.Set(YX,YY,YZ,ZX,ZY,ZZ,WX,WY,WZ).Determinant()

	m.YX = -tmp.Set(XY,XZ,XW,ZY,ZZ,ZW,WY,WZ,WW).Determinant()
	m.YY = tmp.Set(XX,XZ,XW,ZX,ZZ,ZW,WX,WZ,WW).Determinant()
	m.YZ = -tmp.Set(XX,XY,XW,ZX,ZY,ZW,WX,WY,WW).Determinant()
	m.YW = tmp.Set(XX,XY,XZ,ZX,ZY,ZZ,WX,WY,WZ).Determinant()

	m.ZX = tmp.Set(XY,XZ,XW,YY,YZ,YW,WY,WZ,WW).Determinant()
	m.ZY = -tmp.Set(XX,XZ,XW,YX,YZ,YW,WX,WZ,WW).Determinant()
	m.ZZ = tmp.Set(XX,XY,XW,YX,YY,YW,WX,WY,WW).Determinant()
	m.ZW = -tmp.Set(XX,XY,XZ,YX,YY,YZ,WX,WY,WZ).Determinant()

	m.WX = -tmp.Set(XY,XZ,XW,YY,YZ,YW,ZY,ZZ,ZW).Determinant()
	m.WY = tmp.Set(XX,XZ,XW,YX,YZ,YW,ZX,ZZ,ZW).Determinant()
	m.WZ = -tmp.Set(XX,XY,XW,YX,YY,YW,ZX,ZY,ZW).Determinant()
	m.WW = tmp.Set(XX,XY,XZ,YX,YY,YZ,ZX,ZY,ZZ).Determinant()

	m.Transpose()

	m.Scale(1 / determinant)

	return m
}



func (m *Matrix4) Swap(matrix *Matrix4) *Matrix4 {
	tmp := m.XX; m.XX = matrix.XX; matrix.XX = tmp
	tmp = m.XY; m.XY = matrix.XY; matrix.XY = tmp
	tmp = m.XZ; m.XZ = matrix.XZ; matrix.XZ = tmp
	tmp = m.XW; m.XW = matrix.XW; matrix.XW = tmp
	
	tmp = m.YX; m.YX = matrix.YX; matrix.YX = tmp
	tmp = m.YY; m.YY = matrix.YY; matrix.YY = tmp
	tmp = m.YZ; m.YZ = matrix.YZ; matrix.YZ = tmp
	tmp = m.YW; m.YW = matrix.YW; matrix.YW = tmp
	
	tmp = m.ZX; m.ZX = matrix.ZX; matrix.ZX = tmp
	tmp = m.ZY; m.ZY = matrix.ZY; matrix.ZY = tmp
	tmp = m.ZZ; m.ZZ = matrix.ZZ; matrix.ZZ = tmp
	tmp = m.ZW; m.ZW = matrix.ZW; matrix.ZW = tmp

	tmp = m.WX; m.WX = matrix.WX; matrix.WX = tmp
	tmp = m.WY; m.WY = matrix.WY; matrix.WY = tmp
	tmp = m.WZ; m.WZ = matrix.WZ; matrix.WZ = tmp
	tmp = m.WW; m.WW = matrix.WW; matrix.WW = tmp
	return m
}
