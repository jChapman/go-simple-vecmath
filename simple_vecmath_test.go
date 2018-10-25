package simple_vecmath

import "testing"

func TestAdd(t *testing.T) {
    v1 := NewVector3(1, 3, 5)
    v2 := NewVector3(2, 4, 6)
    v3 := v1.AddV(v2)
    if (v3.X != 3 && v3.Y != 7 && v3.Z != 11) {
        t.Fail()
    }
}
