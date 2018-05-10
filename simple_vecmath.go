package simple_vecmath

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
	x, y, z float64
}

func NewVector3(x, y, z float64) *Vector3 {
	return &Vector3{x, y, z}
}

func (v *Vector3) ToString() string {
	return fmt.Sprintf("(%g,%g,%g)", v.x, v.y, v.z)
}

func (v *Vector3) Copy() *Vector3 {
	return NewVector3(v.x, v.y, v.z)
}

func (v *Vector3) Set(x, y, z float64) *Vector3 {
	v.x, v.y, v.z = x, y, z
	return v
}

func (v *Vector3) SetV(vector *Vector3) *Vector3 {
	v.x, v.y, v.z = vector.x, vector.y, vector.z
	return v
}

func (v *Vector3) Add(x, y, z float64) *Vector3 {
	v.x += x
	v.y += y
	v.z += z
	return v
}

func (v *Vector3) AddV(vector *Vector3) *Vector3 {
	v.x += vector.x
	v.y += vector.y
	v.z += vector.z
	return v
}

func (v *Vector3) Subtract(x, y, z float64) *Vector3 {
	v.x -= x
	v.y -= y
	v.z -= z
	return v
}

func (v *Vector3) SubtractV(vector *Vector3) *Vector3 {
	v.x -= vector.x
	v.y -= vector.y
	v.z -= vector.z
	return v
}

func (v *Vector3) SubtractVNoOW(vector *Vector3) *Vector3 {
	return NewVector3(0,0,0).SubtractV2(v,vector)
}

func (v *Vector3) SubtractV2(vector1, vector2 *Vector3) *Vector3 {
	v.x = vector1.x - vector2.x
	v.y = vector1.y - vector2.y
	v.z = vector1.z - vector2.z
	return v
}

func (v *Vector3) Scale(scalar float64) *Vector3 {
	v.x *= scalar
	v.y *= scalar
	v.z *= scalar
	return v
}

func (v *Vector3) ScaleV(vector *Vector3) *Vector3 {
	v.x *= vector.x
	v.y *= vector.y
	v.z *= vector.z
	return v
}

func (v *Vector3) Zero() *Vector3 {
	v.x, v.y, v.z = 0, 0, 0
	return v
}

func (v *Vector3) Length() float64 {
	return math.Sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
}

func (v *Vector3) Length2() float64 {
	return v.x*v.x + v.y*v.y + v.z*v.z
}

func (v *Vector3) Normalize() *Vector3 {
	length := v.Length()
	v.x /= length
	v.y /= length
	v.z /= length
	return v
}

func (v *Vector3) Dot(vector *Vector3) float64 {
	return v.x*vector.x + v.y*vector.y + v.z*vector.z
}

func (v *Vector3) CrossNoOW(vector *Vector3) *Vector3 {
	return NewVector3(0,0,0).Cross2(v,vector)
}

func (v *Vector3) Cross(vector *Vector3) *Vector3 {
	crossx := v.y*vector.z - v.z*vector.y
	crossy := v.z*vector.x - v.x*vector.z
	crossz := v.x*vector.y - v.y*vector.x
	return v.Set(crossx, crossy, crossz)
}

func (v *Vector3) Cross2(vector1 *Vector3, vector2 *Vector3) *Vector3 {	
	crossx := vector1.y*vector2.z - vector1.z*vector2.y
	crossy := vector1.z*vector2.x - vector1.x*vector2.z
	crossz := vector1.x*vector2.y - vector1.y*vector2.x
	return v.Set(crossx, crossy, crossz)
}

func (v *Vector3) Lerp(alpha float64, vector1 *Vector3, vector2 *Vector3) *Vector3 {
	alphaFrac := 1 - alpha
	v.x = alpha * vector1.x + alphaFrac * vector2.x;
	v.y = alpha * vector1.y + alphaFrac * vector2.y;
	v.z = alpha * vector1.z + alphaFrac * vector2.z;
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
	v.x = math.Max(math.Min(v.x, max), min)
	v.y = math.Max(math.Min(v.y, max), min)
	v.z = math.Max(math.Min(v.z, max), min)
	return v
}

func (v *Vector3) EpsilonEquals(vector *Vector3, epsilon float64) bool {
	differencex := math.Abs(v.x - vector.x)
	differencey := math.Abs(v.y - vector.y)
	differencez := math.Abs(v.z - vector.z)

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
	v.x, v.y, v.z = -v.x, -v.y, -v.z
	return v
}

func (v *Vector3) ScaleAdd(scalar float64, vector *Vector3) *Vector3 {
	v.x += scalar * vector.x
	v.y += scalar * vector.y
	v.z += scalar * vector.z
	return v
}

func (v *Vector3) Swap(vector *Vector3) *Vector3 {
	tmp := v.x
	v.x = vector.x
	vector.x = tmp
	tmp = v.y
	v.y = vector.y
	vector.y = tmp
	tmp = v.z
	v.z = vector.z
	vector.z = tmp
	return v
}

func (v *Vector3) Matrix3Multiply(m *Matrix3) *Vector3 {
	newX := v.x*m.xx + v.y*m.xy + v.z*m.xz
	newY := v.x*m.yx + v.y*m.yy + v.z*m.yz
	newZ := v.x*m.zx + v.y*m.zy + v.z*m.zz
	v.x, v.y, v.z = newX, newY, newZ
	return v
}

func (v *Vector3) Matrix4Multiply(w float64, m *Matrix4) *Vector3 {	
	newX := v.x*m.xx + v.y*m.xy + v.z*m.xz + w*m.xw
	newY := v.x*m.yx + v.y*m.yy + v.z*m.yz + w*m.yw
	newZ := v.x*m.zx + v.y*m.zy + v.z*m.zz + w*m.zw
	v.x, v.y, v.z = newX, newY, newZ
	return v
}

// Matrix3  -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

type Matrix3 struct {
	xx, xy, xz float64
	yx, yy, yz float64
	zx, zy, zz float64
}

func NewMatrix3Identity() *Matrix3 {
	m := Matrix3{1, 0, 0,
		0, 1, 0,
		0, 0, 1}
	return &m
}

func NewMatrix3(xx, xy, xz, yx, yy, yz, zx, zy, zz float64) *Matrix3 {
	m := Matrix3{xx, xy, xz,
		yx, yy, yz,
		zx, zy, zz}
	return &m
}

func NewMatrix3V(vx, vy, vz *Vector3) *Matrix3 {
	m := Matrix3{vx.x, vy.x, vz.x,
		vx.y, vy.y, vz.y,
		vx.z, vy.z, vz.z}
	return &m
}

func (m *Matrix3) ToString() string {
	return fmt.Sprintf("( %g,%g,%g | %g,%g,%g | %g,%g,%g )",
		m.xx, m.xy, m.xz,
		m.yx, m.yy, m.yz,
		m.zx, m.zy, m.zz)
}

func (m *Matrix3) Copy() *Matrix3 {
	return &Matrix3{
		m.xx, m.xy, m.xz,
		m.yx, m.yy, m.yz,
		m.zx, m.zy, m.zz}
}

func (m *Matrix3) Identity() *Matrix3 {
	m.xx, m.xy, m.xz = 1, 0, 0
	m.yx, m.yy, m.yz = 0, 1, 0
	m.zx, m.zy, m.zz = 0, 0, 1
	return m
}

func (m *Matrix3) Set(xx, xy, xz, yx, yy, yz, zx, zy, zz float64) *Matrix3 {
	m.xx, m.xy, m.xz = xx, xy, xz
	m.yx, m.yy, m.yz = yx, yy, yz
	m.zx, m.zy, m.zz = zx, zy, zz
	return m
}

func (m *Matrix3) SetV(vx, vy, vz *Vector3) *Matrix3 {
	m.xx, m.xy, m.xz = vx.x, vy.x, vz.x
	m.yx, m.yy, m.yz = vx.y, vy.y, vz.y
	m.zx, m.zy, m.zz = vx.z, vy.z, vz.z
	return m
}

func (m *Matrix3) SetM(matrix *Matrix3) *Matrix3 {
	m.xx, m.xy, m.xz = matrix.xx, matrix.xy, matrix.xz
	m.yx, m.yy, m.yz = matrix.yx, matrix.yy, matrix.yz
	m.zx, m.zy, m.zz = matrix.zx, matrix.zy, matrix.zz
	return m
}

func (m *Matrix3) Determinant() float64 {
	diagonal1 := m.xx*m.yy*m.zz + m.xy*m.yz*m.zx + m.xz*m.yx*m.zy
	diagonal2 := -m.zx*m.yy*m.xz - m.zy*m.yz*m.xx - m.zz*m.yx*m.xy
	return diagonal1 + diagonal2
}

func (m *Matrix3) Invert() *Matrix3 {

	determinant := m.Determinant()

	if EpsilonEquals(determinant, 0, ePSILON) {
		panic("Matrix does not have an inverse")
	}

	iDet := 1 / determinant

	nxx := (m.yy*m.zz - m.yz*m.zy) * iDet
	nyx := -(m.yx*m.zz - m.yz*m.zx) * iDet
	nzx := (m.yx*m.zy - m.yy*m.zx) * iDet
	nxy := -(m.xy*m.zz - m.xz*m.zy) * iDet
	nyy := (m.xx*m.zz - m.xz*m.zx) * iDet
	nzy := -(m.xx*m.zy - m.xy*m.zx) * iDet
	nxz := (m.xy*m.yz - m.xz*m.yy) * iDet
	nyz := -(m.xx*m.yz - m.xz*m.yx) * iDet
	nzz := (m.xx*m.yy - m.xy*m.yx) * iDet

	m.xx, m.xy, m.xz = nxx, nxy, nxz
	m.yx, m.yy, m.yz = nyx, nyy, nyz
	m.zx, m.zy, m.zz = nzx, nzy, nzz

	return m
}

func (m *Matrix3) Multiply(m3 *Matrix3) *Matrix3 {
	nxx := m.xx*m3.xx + m.xy*m3.yx + m.xz*m3.zx
	nxy := m.xx*m3.xy + m.xy*m3.yy + m.xz*m3.zy
	nxz := m.xx*m3.xz + m.xy*m3.yz + m.xz*m3.zz
	nyx := m.yx*m3.xx + m.yy*m3.yx + m.yz*m3.zx
	nyy := m.yx*m3.xy + m.yy*m3.yy + m.yz*m3.zy
	nyz := m.yx*m3.xz + m.yy*m3.yz + m.yz*m3.zz
	nzx := m.zx*m3.xx + m.zy*m3.yx + m.zz*m3.zx
	nzy := m.zx*m3.xy + m.zy*m3.yy + m.zz*m3.zy
	nzz := m.zx*m3.xz + m.zy*m3.yz + m.zz*m3.zz

	m.xx, m.xy, m.xz = nxx, nxy, nxz
	m.yx, m.yy, m.yz = nyx, nyy, nyz
	m.zx, m.zy, m.zz = nzx, nzy, nzz

	return m
}

func (m *Matrix3) MultiplyV(vector *Vector3) *Matrix3 {
	m.xx *= vector.x; m.xy *= vector.y; m.xz *= vector.z
	m.yx *= vector.x; m.yy *= vector.y; m.yz *= vector.z
	m.zx *= vector.x; m.zy *= vector.y; m.zz *= vector.z
	return m
}

func (m *Matrix3) Scale(scalar float64) *Matrix3 {
	m.xx *= scalar; m.xy *= scalar; m.xz *= scalar
	m.yx *= scalar; m.yy *= scalar; m.yz *= scalar
	m.zx *= scalar; m.zy *= scalar; m.zz *= scalar
	return m
}

func (m *Matrix3) Add(m3 *Matrix3) *Matrix3 {	
	m.xx += m3.xx; m.xy += m3.xy; m.xz += m3.xz
	m.yx += m3.yx; m.yy += m3.yy; m.yz += m3.yz
	m.zx += m3.zx; m.zy += m3.zy; m.zz += m3.zz
	return m
}

func (m *Matrix3) Subtract(m3 *Matrix3) *Matrix3 {	
	m.xx -= m3.xx; m.xy -= m3.xy; m.xz -= m3.xz
	m.yx -= m3.yx; m.yy -= m3.yy; m.yz -= m3.yz
	m.zx -= m3.zx; m.zy -= m3.zy; m.zz -= m3.zz
	return m
}

func (m *Matrix3) IsInvertable() bool {
	return !EpsilonEquals(m.Determinant(), 0, ePSILON)
}

func (m *Matrix3) Swap(matrix *Matrix3) *Matrix3 {
	tmp := m.xx; m.xx = matrix.xx; matrix.xx = tmp
	tmp = m.xy; m.xy = matrix.xy; matrix.xy = tmp
	tmp = m.xz; m.xz = matrix.xz; matrix.xz = tmp
	tmp = m.yx; m.yx = matrix.yx; matrix.yx = tmp
	tmp = m.yy; m.yy = matrix.yy; matrix.yy = tmp
	tmp = m.yz; m.yz = matrix.yz; matrix.yz = tmp
	tmp = m.zx; m.zx = matrix.zx; matrix.zx = tmp
	tmp = m.zy; m.zy = matrix.zy; matrix.zy = tmp
	tmp = m.zz; m.zz = matrix.zz; matrix.zz = tmp
	return m
}

func (m *Matrix3) Transpose() *Matrix3 {
	tmp := m.xy; m.xy = m.yx; m.yx = tmp
	tmp = m.xz; m.xz = m.zx; m.zx = tmp
	tmp = m.yz; m.yz = m.zy; m.zy = tmp
	return m
}

// Matrix4  -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

type Matrix4 struct {
	xx, xy, xz, xw float64
	yx, yy, yz, yw float64
	zx, zy, zz, zw float64
	wx, wy, wz, ww float64
}

func NewMatrix4Identity() *Matrix4 {
	m := Matrix4{
    		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1}
	return &m
}

func NewMatrix4(xx, xy, xz, xw, yx, yy, yz, yw, zx, zy, zz, zw, wx, wy, wz, ww float64) *Matrix4 {
	m := Matrix4{
    		xx, xy, xz, xw,
		yx, yy, yz, yw,
		zx, zy, zz, zw,
		wx, wy, wz, ww}
	return &m
}

func NewMatrix4V(vx, vy, vz, vw *Vector3, wx, wy, wz, ww float64) *Matrix4 {
	m := Matrix4{
    		vx.x, vy.x, vz.x, vw.x,
		vx.y, vy.y, vz.y, vw.y,
		vx.z, vy.z, vz.z, vw.z,
		wx  , wy  , wz  , ww  }
	return &m
}

func (m *Matrix4) ToString() string {
	return fmt.Sprintf("( %g,%g,%g,%g | %g,%g,%g,%g | %g,%g,%g,%g | %g,%g,%g,%g )",
		m.xx, m.xy, m.xz, m.xw,
		m.yx, m.yy, m.yz, m.yw,
		m.zx, m.zy, m.zz, m.zw,
		m.wx, m.wy, m.wz, m.ww)
}

func (m *Matrix4) Copy() *Matrix4 {
	return &Matrix4{
		m.xx, m.xy, m.xz, m.xw,
		m.yx, m.yy, m.yz, m.yw,
		m.zx, m.zy, m.zz, m.zw,
		m.wx, m.wy, m.wz, m.ww}
}

func (m *Matrix4) Identity() *Matrix4 {
	m.xx, m.xy, m.xz, m.xw = 1, 0, 0, 0
	m.yx, m.yy, m.yz, m.yw = 0, 1, 0, 0
	m.zx, m.zy, m.zz, m.zw = 0, 0, 1, 0
	m.wx, m.wy, m.wz, m.ww = 0, 0, 0, 1
	return m
}

func (m *Matrix4) Set(xx, xy, xz, xw, yx, yy, yz, yw, zx, zy, zz, zw, wx, wy, wz, ww float64) *Matrix4 {
	m.xx, m.xy, m.xz, m.xw = xx, xy, xz, xw
	m.yx, m.yy, m.yz, m.yw = yx, yy, yz, yw
	m.zx, m.zy, m.zz, m.zw = zx, zy, zz, zw
	m.wx, m.wy, m.wz, m.ww = wx, wy, wz, ww
	return m
}

func (m *Matrix4) SetV(vx, vy, vz, vw *Vector3, wx, wy, wz, ww float64) *Matrix4 {
	m.xx, m.xy, m.xz, m.xw = vx.x, vy.x, vz.x, vw.x
	m.yx, m.yy, m.yz, m.yw = vx.y, vy.y, vz.y, vw.y
	m.zx, m.zy, m.zz, m.zw = vx.z, vy.z, vz.z, vw.z
	m.wx, m.wy, m.wz, m.ww = wx  , wy  , wz  , ww
	return m
}

func (m *Matrix4) SetM(matrix *Matrix4) *Matrix4 {
	m.xx, m.xy, m.xz, m.xw = matrix.xx, matrix.xy, matrix.xz, matrix.xw
	m.yx, m.yy, m.yz, m.yw = matrix.yx, matrix.yy, matrix.yz, matrix.yw
	m.zx, m.zy, m.zz, m.zw = matrix.zx, matrix.zy, matrix.zz, matrix.zw
	m.wx, m.wy, m.wz, m.ww = matrix.wx, matrix.wy, matrix.wz, matrix.ww
	return m
}

func (m *Matrix4) Determinant() float64 {

	submatrix := NewMatrix3(m.yy,m.yz,m.yw,m.zy,m.zz,m.zw,m.wy,m.wz,m.ww)
	det1 := submatrix.Determinant()

	submatrix.Set(m.yx,m.yz,m.yw,m.zx,m.zz,m.zw,m.wx,m.wz,m.ww)
	det2 := submatrix.Determinant()

	submatrix.Set(m.yx,m.yy,m.yw,m.zx,m.zy,m.zw,m.wx,m.wy,m.ww)
	det3 := submatrix.Determinant()

	submatrix.Set(m.yx,m.yy,m.yz,m.zx,m.zy,m.zz,m.wx,m.wy,m.wz)
	det4 := submatrix.Determinant()

	return m.xx * det1 - m.xy * det2 + m.xz * det3 - m.xw * det4
}

func (m *Matrix4) MultiplyV(vector *Vector3, w float64) *Matrix4 {
	m.xx *= vector.x; m.xy *= vector.y; m.xz *= vector.z; m.xw *= w
	m.yx *= vector.x; m.yy *= vector.y; m.yz *= vector.z; m.yw *= w
	m.zx *= vector.x; m.zy *= vector.y; m.zz *= vector.z; m.zw *= w
	m.wx *= vector.x; m.wy *= vector.y; m.wz *= vector.z; m.ww *= w
	return m
}

func (m *Matrix4) Multiply(m4 *Matrix4) *Matrix4 {
	nxx := m.xx*m4.xx + m.xy*m4.yx + m.xz*m4.zx + m.xw*m4.wx
	nxy := m.xx*m4.xy + m.xy*m4.yy + m.xz*m4.zy + m.xw*m4.wy
	nxz := m.xx*m4.xz + m.xy*m4.yz + m.xz*m4.zz + m.xw*m4.wz
	nxw := m.xx*m4.xw + m.xy*m4.yw + m.xz*m4.zw + m.xw*m4.ww
	
	nyx := m.yx*m4.xx + m.yy*m4.yx + m.yz*m4.zx + m.yw*m4.wx
	nyy := m.yx*m4.xy + m.yy*m4.yy + m.yz*m4.zy + m.yw*m4.wy
	nyz := m.yx*m4.xz + m.yy*m4.yz + m.yz*m4.zz + m.yw*m4.wz
	nyw := m.yx*m4.xw + m.yy*m4.yw + m.yz*m4.zw + m.yw*m4.ww

	nzx := m.zx*m4.xx + m.zy*m4.yx + m.zz*m4.zx + m.zw*m4.wx
	nzy := m.zx*m4.xy + m.zy*m4.yy + m.zz*m4.zy + m.zw*m4.wy
	nzz := m.zx*m4.xz + m.zy*m4.yz + m.zz*m4.zz + m.zw*m4.wz
	nzw := m.zx*m4.xw + m.zy*m4.yw + m.zz*m4.zw + m.zw*m4.ww

	nwx := m.wx*m4.xx + m.wy*m4.yx + m.wz*m4.zx + m.ww*m4.wx
	nwy := m.wx*m4.xy + m.wy*m4.yy + m.wz*m4.zy + m.ww*m4.wy
	nwz := m.wx*m4.xz + m.wy*m4.yz + m.wz*m4.zz + m.ww*m4.wz
	nww := m.wx*m4.xw + m.wy*m4.yw + m.wz*m4.zw + m.ww*m4.ww

	m.xx, m.xy, m.xz, m.xw = nxx, nxy, nxz, nxw
	m.yx, m.yy, m.yz, m.yw = nyx, nyy, nyz, nyw
	m.zx, m.zy, m.zz, m.zw = nzx, nzy, nzz, nzw
	m.wx, m.wy, m.wz, m.ww = nwx, nwy, nwz, nww

	return m
}

func (m *Matrix4) Scale(scalar float64) *Matrix4 {
	m.xx *= scalar; m.xy *= scalar; m.xz *= scalar; m.xw *= scalar
	m.yx *= scalar; m.yy *= scalar; m.yz *= scalar; m.yw *= scalar
	m.zx *= scalar; m.zy *= scalar; m.zz *= scalar; m.zw *= scalar
	m.wx *= scalar; m.wy *= scalar; m.wz *= scalar; m.ww *= scalar
	return m
}

func (m *Matrix4) Add(m4 *Matrix4) *Matrix4 {	
	m.xx += m4.xx; m.xy += m4.xy; m.xz += m4.xz; m.xw += m4.xw
	m.yx += m4.yx; m.yy += m4.yy; m.yz += m4.yz; m.yw += m4.yw
	m.zx += m4.zx; m.zy += m4.zy; m.zz += m4.zz; m.zw += m4.zw
	m.wx += m4.wx; m.wy += m4.wy; m.wz += m4.wz; m.ww += m4.ww
	return m
}

func (m *Matrix4) Subtract(m4 *Matrix4) *Matrix4 {	
	m.xx -= m4.xx; m.xy -= m4.xy; m.xz -= m4.xz; m.xw -= m4.xw
	m.yx -= m4.yx; m.yy -= m4.yy; m.yz -= m4.yz; m.yw -= m4.yw
	m.zx -= m4.zx; m.zy -= m4.zy; m.zz -= m4.zz; m.zw -= m4.zw
	m.wx -= m4.wx; m.wy -= m4.wy; m.wz -= m4.wz; m.ww -= m4.ww
	return m
}

func (m *Matrix4) IsInvertable() bool {
	return !EpsilonEquals(m.Determinant(), 0, ePSILON)
}

func (m *Matrix4) Transpose() *Matrix4 {
	tmp := m.xy; m.xy = m.yx; m.yx = tmp
	tmp = m.xz; m.xz = m.zx; m.zx = tmp
	tmp = m.xw; m.xw = m.wx; m.wx = tmp
	tmp = m.yz; m.yz = m.zy; m.zy = tmp
	tmp = m.yw; m.yw = m.wy; m.wy = tmp
	tmp = m.zw; m.zw = m.wz; m.wz = tmp
	return m
}

func (m *Matrix4) Invert() *Matrix4 {

	determinant := m.Determinant()

	if EpsilonEquals(determinant, 0, ePSILON) {
		panic("Matrix does not have an inverse")
	}

	xx := m.xx; xy := m.xy; xz := m.xz; xw := m.xw
	yx := m.yx; yy := m.yy; yz := m.yz; yw := m.yw
	zx := m.zx; zy := m.zy; zz := m.zz; zw := m.zw
	wx := m.wx; wy := m.wy; wz := m.wz; ww := m.ww

	tmp:= NewMatrix3(0,0,0,0,0,0,0,0,0)
	
	//diagonal1 := m.xx*m.yy*m.zz + m.xy*m.yz*m.zx + m.xz*m.yx*m.zy
	//diagonal2 := -m.zx*m.yy*m.xz - m.zy*m.yz*m.xx - m.zz*m.yx*m.xy

	m.xx = yy*zz*wz + yz*zw*wy + yw*zy*wz - wy*zz*yw - wz*zw*yy - ww*zy*yz  //tmp.Set(yy,yz,yw,zy,zz,zw,wy,wz,ww).Determinant()
	m.xy = -tmp.Set(yx,yz,yw,zx,zz,zw,wx,wz,ww).Determinant()
	m.xz = tmp.Set(yx,yy,yw,zx,zy,zw,wx,wy,ww).Determinant()
	m.xw = -tmp.Set(yx,yy,yz,zx,zy,zz,wx,wy,wz).Determinant()

	m.yx = -tmp.Set(xy,xz,xw,zy,zz,zw,wy,wz,ww).Determinant()
	m.yy = tmp.Set(xx,xz,xw,zx,zz,zw,wx,wz,ww).Determinant()
	m.yz = -tmp.Set(xx,xy,xw,zx,zy,zw,wx,wy,ww).Determinant()
	m.yw = tmp.Set(xx,xy,xz,zx,zy,zz,wx,wy,wz).Determinant()

	m.zx = tmp.Set(xy,xz,xw,yy,yz,yw,wy,wz,ww).Determinant()
	m.zy = -tmp.Set(xx,xz,xw,yx,yz,yw,wx,wz,ww).Determinant()
	m.zz = tmp.Set(xx,xy,xw,yx,yy,yw,wx,wy,ww).Determinant()
	m.zw = -tmp.Set(xx,xy,xz,yx,yy,yz,wx,wy,wz).Determinant()

	m.wx = -tmp.Set(xy,xz,xw,yy,yz,yw,zy,zz,zw).Determinant()
	m.wy = tmp.Set(xx,xz,xw,yx,yz,yw,zx,zz,zw).Determinant()
	m.wz = -tmp.Set(xx,xy,xw,yx,yy,yw,zx,zy,zw).Determinant()
	m.ww = tmp.Set(xx,xy,xz,yx,yy,yz,zx,zy,zz).Determinant()

	m.Transpose()

	m.Scale(1 / determinant)

	return m
}



func (m *Matrix4) Swap(matrix *Matrix4) *Matrix4 {
	tmp := m.xx; m.xx = matrix.xx; matrix.xx = tmp
	tmp = m.xy; m.xy = matrix.xy; matrix.xy = tmp
	tmp = m.xz; m.xz = matrix.xz; matrix.xz = tmp
	tmp = m.xw; m.xw = matrix.xw; matrix.xw = tmp
	
	tmp = m.yx; m.yx = matrix.yx; matrix.yx = tmp
	tmp = m.yy; m.yy = matrix.yy; matrix.yy = tmp
	tmp = m.yz; m.yz = matrix.yz; matrix.yz = tmp
	tmp = m.yw; m.yw = matrix.yw; matrix.yw = tmp
	
	tmp = m.zx; m.zx = matrix.zx; matrix.zx = tmp
	tmp = m.zy; m.zy = matrix.zy; matrix.zy = tmp
	tmp = m.zz; m.zz = matrix.zz; matrix.zz = tmp
	tmp = m.zw; m.zw = matrix.zw; matrix.zw = tmp

	tmp = m.wx; m.wx = matrix.wx; matrix.wx = tmp
	tmp = m.wy; m.wy = matrix.wy; matrix.wy = tmp
	tmp = m.wz; m.wz = matrix.wz; matrix.wz = tmp
	tmp = m.ww; m.ww = matrix.ww; matrix.ww = tmp
	return m
}
