/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Wenzel Jakob
	@author Nico Schertler
*/

#include "field.h"
#include "common.h"

#include <Eigen/Geometry>

using namespace osr;

static const Float sqrt_3_over_4 = 0.866025403784439f;

Vector3f rotate180(const Vector3f &q, const Vector3f &/* unused */) {
    return -q;
}

Vector3f rotate180_by(const Vector3f &q, const Vector3f &/* unused */, int amount) {
    return (amount & 1) ? Vector3f(-q) : q;
}

Vector2i rshift180(Vector2i shift, int amount) {
    if (amount & 1)
        shift = -shift;
    return shift;
}

Vector3f rotate90(const Vector3f &q, const Vector3f &n) {
    return n.cross(q);
}

Vector3f rotate90_by(const Vector3f &q, const Vector3f &n, int amount) {
    return ((amount & 1) ? (n.cross(q)) : q) * (amount < 2 ? 1.0f : -1.0f);
}

Vector2i rshift90(Vector2i shift, int amount) {
    if (amount & 1)
        shift = Vector2i(-shift.y(), shift.x());
    if (amount >= 2)
        shift = -shift;
    return shift;
}

Vector3f rotate60(const Vector3f &d, const Vector3f &n) {
    return sqrt_3_over_4 * n.cross(d) + 0.5f*(d + n * n.dot(d));
}

Vector2i rshift60(Vector2i shift, int amount) {
    for (int i=0; i<amount; ++i)
        shift = Vector2i(-shift.y(), shift.x() + shift.y());
    return shift;
}

Vector3f rotate60_by(const Vector3f &d, const Vector3f &n, int amount) {
    switch (amount) {
        case 0: return d;
        case 1: return rotate60(d, n);
        case 2: return -rotate60(d, -n);
        case 3: return -d;
        case 4: return -rotate60(d, n);
        case 5: return rotate60(d, -n);
    }
    throw std::runtime_error("rotate60: invalid argument");
}

Vector3f rotate_vector_into_plane(Vector3f q, const Vector3f &source_normal, const Vector3f &target_normal) {
    const Float cosTheta = source_normal.dot(target_normal);
    if (cosTheta < 0.9999f) {
        Vector3f axis = source_normal.cross(target_normal);
        q = q * cosTheta + axis.cross(q) +
             axis * (axis.dot(q) * (1.0f - cosTheta) / axis.dot(axis));
    }
    return q;
}

inline Vector3f middle_point(const Vector3f &p0, const Vector3f &n0, const Vector3f &p1, const Vector3f &n1) {
    /* How was this derived?
     *
     * Minimize \|x-p0\|^2 + \|x-p1\|^2, where
     * dot(n0, x) == dot(n0, p0)
     * dot(n1, x) == dot(n1, p1)
     *
     * -> Lagrange multipliers, set derivative = 0
     *  Use first 3 equalities to write x in terms of
     *  lambda_1 and lambda_2. Substitute that into the last
     *  two equations and solve for the lambdas. Finally,
     *  add a small epsilon term to avoid issues when n1=n2.
     */
    Float n0p0 = n0.dot(p0), n0p1 = n0.dot(p1),
          n1p0 = n1.dot(p0), n1p1 = n1.dot(p1),
          n0n1 = n0.dot(n1),
          denom = 1.0f / (1.0f - n0n1*n0n1 + 1e-4f),
          lambda_0 = 2.0f*(n0p1 - n0p0 - n0n1*(n1p0 - n1p1))*denom,
          lambda_1 = 2.0f*(n1p0 - n1p1 - n0n1*(n0p1 - n0p0))*denom;

    return 0.5f * (p0 + p1) - 0.25f * (n0 * lambda_0 + n1 * lambda_1);
}

IOrientationFieldTraits::IOrientationFieldTraits(int rosy)
	: _rosy(rosy)
{ }


template<int RoSy>
OrientationFieldTraits<RoSy>::OrientationFieldTraits()
	: IOrientationFieldTraits(RoSy)
{ }

template <>
void OrientationFieldTraits<2>::findCompatible(const Vector3f &q0, const Vector3f &n0, const Vector3f &q1, const Vector3f &n1, Vector3f& compat1, Vector3f& compat2)
{
	compat1 = q0;
	compat2 = q1 * signum(q0.dot(q1));
}

template <>
void OrientationFieldTraits<2>::findCompatible(const Vector3f &dirField0, const Vector3f &normal0, const Matrix3Xf& principalDirections, Matrix3Xf& compatOut, VectorXf& scoreOut)
{
	float dot = dirField0.dot(principalDirections.col(0));
	compatOut.col(0) = dirField0 * signum(dot);
	scoreOut(0) = std::abs(dot);
}

template <>
void OrientationFieldTraits<2>::getPrincipalDirections(const Vector3f& dirField, const Vector3f& normal, Matrix3Xf& outDirections)
{
	outDirections.col(0) = dirField;	
}

template <>
void OrientationFieldTraits<4>::getPrincipalDirections(const Vector3f& dirField, const Vector3f& normal, Matrix3Xf& outDirections)
{
	outDirections.col(0) = dirField;
	outDirections.col(1) = normal.cross(dirField);
}

template <>
void OrientationFieldTraits<4>::findCompatible(const Vector3f &q0, const Vector3f &n0, const Vector3f &q1, const Vector3f &n1, Vector3f& compat1, Vector3f& compat2)
{
	Matrix3Xf A(3, 2), B(3, 2);
	getPrincipalDirections(q0, n0, A);
	getPrincipalDirections(q1, n1, B);    

    Float best_score = -std::numeric_limits<Float>::infinity();
    int best_a = 0, best_b = 0;

    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            Float score = std::abs(A.col(i).dot(B.col(j)));
            if (score > best_score) {
                best_a = i;
                best_b = j;
                best_score = score;
            }
        }
    }

    const Float dp = A.col(best_a).dot(B.col(best_b));
	compat1 = A.col(best_a);
	compat2 = B.col(best_b) * signum(dp);
}

template <>
void OrientationFieldTraits<4>::findCompatible(const Vector3f &dirField0, const Vector3f &normal0, const Matrix3Xf& principalDirections, Matrix3Xf& compatOut, VectorXf& scoreOut)
{
	Matrix3Xf B(3, 2);
	getPrincipalDirections(dirField0, normal0, B);	

	for (int i = 0; i < 2; ++i) 
	{
		Float best_score = -std::numeric_limits<Float>::infinity();
		int best = 0;

		for (int j = 0; j < 2; ++j)
		{
			Float score = std::abs(principalDirections.col(i).dot(B.col(j)));
			if (score > best_score) 
			{
				best = j;
				best_score = score;
			}
		}

		const Float dp = principalDirections.col(i).dot(B.col(best));
		compatOut.col(i) = B.col(best) * signum(dp);
		scoreOut(i) = best_score;
	}	
}

template <>
void OrientationFieldTraits<6>::getPrincipalDirections(const Vector3f& dirField, const Vector3f& normal, Matrix3Xf& outDirections)
{
	outDirections.col(0) = rotate60(dirField, -normal);
	outDirections.col(1) = dirField;
	outDirections.col(2) = rotate60(dirField, normal);
}

template <>
void OrientationFieldTraits<6>::findCompatible(const Vector3f &q0, const Vector3f &n0, const Vector3f &q1, const Vector3f &n1, Vector3f& compat1, Vector3f& compat2)
{
	Matrix3Xf A(3, 3), B(3, 3);
	getPrincipalDirections(q0, n0, A);
	getPrincipalDirections(q1, n1, B);

    Float best_score = -std::numeric_limits<Float>::infinity();
    int best_a = 0, best_b = 0;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Float score = std::abs(A.col(i).dot(B.col(j)));
            if (score > best_score) {
                best_a = i;
                best_b = j;
                best_score = score;
            }
        }
    }

    const Float dp = A.col(best_a).dot(B.col(best_b));
	compat1 = A.col(best_a);
	compat2 = B.col(best_b) * signum(dp);
}

template <>
void OrientationFieldTraits<6>::findCompatible(const Vector3f &dirField0, const Vector3f &normal0, const Matrix3Xf& principalDirections, Matrix3Xf& compatOut, VectorXf& scoreOut)
{
	Matrix3Xf B(3, 3);
	getPrincipalDirections(dirField0, normal0, B);

	for (int i = 0; i < 3; ++i) 
	{
		Float best_score = -std::numeric_limits<Float>::infinity();
		int best = 0;
		for (int j = 0; j < 3; ++j) 
		{
			Float score = std::abs(principalDirections.col(i).dot(B.col(j)));
			if (score > best_score)
			{
				best = j;
				best_score = score;
			}
		}
		const Float dp = principalDirections.col(i).dot(B.col(best));
		compatOut.col(i) = B.col(best) * signum(dp);
		scoreOut(i) = best_score;
	}
}

IOrientationFieldTraits * osr::getOrientationFieldTraits(int roSy)
{
	if (roSy == 2) {
		return new OrientationFieldTraits<2>();
	}
	else if (roSy == 4) {
		return new OrientationFieldTraits<4>();		
	}
	else if (roSy == 6) {
		return new OrientationFieldTraits<6>();		
	}
	else {
		throw std::runtime_error("Invalid rotation symmetry type " + std::to_string(roSy) + "!");
	}
}

// ---- Position Field ----

IPositionFieldTraits::IPositionFieldTraits(int posy)
	:_posy(posy)
{
}

template<int PoSy>
PositionFieldTraits<PoSy>::PositionFieldTraits()
	: IPositionFieldTraits(PoSy)
{
}

IPositionFieldTraits * osr::getPositionFieldTraits(int poSy)
{
	if (poSy == 4) {
		return new PositionFieldTraits<4>();
	}
	else if (poSy == 6) {
		return new PositionFieldTraits<6>();
	}
	else {
		throw std::runtime_error("Invalid position symmetry type " + std::to_string(poSy) + "!");
	}
}


inline Vector2i position_round_index_3(const Vector3f &o, const Vector3f &q,
	const Vector3f &n, const Vector3f &p,
	Float scale, Float inv_scale)
{
	Vector3f t = rotate60(q, n);
	Vector3f d = p - o;
	Float dpq = q.dot(d), dpt = t.dot(d);
	int u = (int)std::floor((4 * dpq - 2 * dpt) * (1.0f / 3.0f) * inv_scale);
	int v = (int)std::floor((-2 * dpq + 4 * dpt) * (1.0f / 3.0f) * inv_scale);

	Float best_cost = std::numeric_limits<Float>::infinity();
	int best_i = -1;

	for (int i = 0; i<4; ++i) {
		Vector3f ot = o + (q*(u + (i & 1)) + t * (v + ((i & 2) >> 1))) * scale;
		Float cost = (ot - p).squaredNorm();
		if (cost < best_cost) {
			best_i = i;
			best_cost = cost;
		}
	}

	return Vector2i(
		u + (best_i & 1), v + ((best_i & 2) >> 1)
	);
}

inline Vector2i position_round_index_4(const Vector3f &o, const Vector3f &q,
	const Vector3f &n, const Vector3f &p,
	Float /* unused */, Float inv_scale) 
{
	Vector3f t = n.cross(q);
	Vector3f d = p - o;
	return Vector2i(
		(int)std::round(q.dot(d) * inv_scale),
		(int)std::round(t.dot(d) * inv_scale));
}

inline Vector3f position_floor_3(const Vector3f &o, const Vector3f &q,
	const Vector3f &n, const Vector3f &p,
	Float scale, Float inv_scale) {
	Vector3f t = rotate60(q, n);
	Vector3f d = p - o;
	Float dpq = q.dot(d), dpt = t.dot(d);
	Float u = std::floor((4 * dpq - 2 * dpt) * (1.0f / 3.0f) * inv_scale);
	Float v = std::floor((-2 * dpq + 4 * dpt) * (1.0f / 3.0f) * inv_scale);

	return o + (q*u + t*v) * scale;
}

inline Vector3f position_floor_4(const Vector3f &o, const Vector3f &q,
	const Vector3f &n, const Vector3f &p,
	Float scale, Float inv_scale) {
	Vector3f t = n.cross(q);
	Vector3f d = p - o;
	return o +
		q * std::floor(q.dot(d) * inv_scale) * scale +
		t * std::floor(t.dot(d) * inv_scale) * scale;
}

inline Vector2i position_floor_index_3(const Vector3f &o, const Vector3f &q,
	const Vector3f &n, const Vector3f &p,
	Float /* scale */, Float inv_scale) {
	Vector3f t = rotate60(q, n);
	Vector3f d = p - o;
	Float dpq = q.dot(d), dpt = t.dot(d);
	int u = (int)std::floor((4 * dpq - 2 * dpt) * (1.0f / 3.0f) * inv_scale);
	int v = (int)std::floor((-2 * dpq + 4 * dpt) * (1.0f / 3.0f) * inv_scale);

	return Vector2i(u, v);
}

inline Vector2i position_floor_index_4(const Vector3f &o, const Vector3f &q,
	const Vector3f &n, const Vector3f &p,
	Float /* unused */, Float inv_scale) {
	Vector3f t = n.cross(q);
	Vector3f d = p - o;
	return Vector2i(
		(int)std::floor(q.dot(d) * inv_scale),
		(int)std::floor(t.dot(d) * inv_scale));
}

template<>
Vector3f PositionFieldTraits<4>::positionRound(const Vector3f &o, const Vector3f &q,
	const Vector3f &n, const Vector3f &p,
	Float scale, Float inv_scale)
{
	Vector3f t = n.cross(q);
	Vector3f d = p - o;
	return o +
		q * std::round(q.dot(d) * inv_scale) * scale +
		t * std::round(t.dot(d) * inv_scale) * scale;
}

template<>
Vector3f PositionFieldTraits<6>::positionRound(const Vector3f &o, const Vector3f &q,
	const Vector3f &n, const Vector3f &p,
	Float scale, Float inv_scale)
{
	Vector3f t = rotate60(q, n);
	Vector3f d = p - o;

	Float dpq = q.dot(d), dpt = t.dot(d);
	Float u = std::floor((4 * dpq - 2 * dpt) * (1.0f / 3.0f) * inv_scale);
	Float v = std::floor((-2 * dpq + 4 * dpt) * (1.0f / 3.0f) * inv_scale);

	Float best_cost = std::numeric_limits<Float>::infinity();
	int best_i = -1;

	for (int i = 0; i<4; ++i) {
		Vector3f ot = o + (q*(u + (i & 1)) + t*(v + ((i & 2) >> 1))) * scale;
		Float cost = (ot - p).squaredNorm();
		if (cost < best_cost) {
			best_i = i;
			best_cost = cost;
		}
	}

	return o + (q*(u + (best_i & 1)) + t*(v + ((best_i & 2) >> 1))) * scale;
}

template <>
void PositionFieldTraits<4>::findCompatible(
        const Vector3f &p0, const Vector3f &n0, const Vector3f &q0, const Vector3f &o0,
        const Vector3f &p1, const Vector3f &n1, const Vector3f &q1, const Vector3f &o1,
        Float scale, Float inv_scale, Vector3f& compat1, Vector3f& compat2)
{

    Vector3f t0 = n0.cross(q0), t1 = n1.cross(q1);
    Vector3f middle = middle_point(p0, n0, p1, n1);
    Vector3f o0p = position_floor_4(o0, q0, n0, middle, scale, inv_scale);
    Vector3f o1p = position_floor_4(o1, q1, n1, middle, scale, inv_scale);

    Float best_cost = std::numeric_limits<Float>::infinity();
    int best_i = -1, best_j = -1;

    for (int i=0; i<4; ++i)
	{
        Vector3f o0t = o0p + (q0 * (i&1) + t0 * ((i&2) >> 1)) * scale;
        for (int j=0; j<4; ++j)
		{
            Vector3f o1t = o1p + (q1 * (j&1) + t1 * ((j&2) >> 1)) * scale;
            Float cost = (o0t-o1t).squaredNorm();

            if (cost < best_cost) 
			{
                best_i = i;
                best_j = j;
                best_cost = cost;
            }
        }
    }

	compat1 = o0p + (q0 * (best_i & 1) + t0 * ((best_i & 2) >> 1)) * scale;
	compat2 = o1p + (q1 * (best_j & 1) + t1 * ((best_j & 2) >> 1)) * scale;
}

template <>
std::pair<Vector2i, Vector2i> PositionFieldTraits<4>::findCompatibleIndex(
        const Vector3f &p0, const Vector3f &n0, const Vector3f &q0, const Vector3f &o0,
        const Vector3f &p1, const Vector3f &n1, const Vector3f &q1, const Vector3f &o1,
        Float scale, Float inv_scale, Float* error) {
    Vector3f t0 = n0.cross(q0), t1 = n1.cross(q1);
    Vector3f middle = middle_point(p0, n0, p1, n1);
    Vector2i o0p = position_floor_index_4(o0, q0, n0, middle, scale, inv_scale);
    Vector2i o1p = position_floor_index_4(o1, q1, n1, middle, scale, inv_scale);

    Float best_cost = std::numeric_limits<Float>::infinity();
    int best_i = -1, best_j = -1;

    for (int i=0; i<4; ++i) {
        Vector3f o0t = o0 + (q0 * ((i&1)+o0p[0]) + t0 * (((i&2) >> 1) + o0p[1])) * scale;
        for (int j=0; j<4; ++j) {
            Vector3f o1t = o1 + (q1 * ((j&1)+o1p[0]) + t1 * (((j&2) >> 1) + o1p[1])) * scale;
            Float cost = (o0t-o1t).squaredNorm();

            if (cost < best_cost) {
                best_i = i;
                best_j = j;
                best_cost = cost;
            }
        }
    }

	if (error)
		*error = best_cost;

    return std::make_pair(
        Vector2i((best_i & 1) + o0p[0], ((best_i & 2) >> 1) + o0p[1]),
        Vector2i((best_j & 1) + o1p[0], ((best_j & 2) >> 1) + o1p[1]));
}

template <>
void PositionFieldTraits<6>::findCompatible(
        const Vector3f &p0, const Vector3f &n0, const Vector3f &q0, const Vector3f &_o0,
        const Vector3f &p1, const Vector3f &n1, const Vector3f &q1, const Vector3f &_o1,
        Float scale, Float inv_scale, Vector3f& compat1, Vector3f& compat2) {
    Vector3f middle = middle_point(p0, n0, p1, n1);
    Vector3f o0 = position_floor_3(_o0, q0, n0, middle, scale, inv_scale);
    Vector3f o1 = position_floor_3(_o1, q1, n1, middle, scale, inv_scale);

    Vector3f t0 = rotate60(q0, n0), t1 = rotate60(q1, n1);
    Float best_cost = std::numeric_limits<Float>::infinity();
    int best_i = -1, best_j = -1;
    for (int i=0; i<4; ++i) {
        Vector3f o0t = o0 + (q0*(i&1) + t0*((i&2)>>1)) * scale;
        for (int j=0; j<4; ++j) {
            Vector3f o1t = o1 + (q1*(j&1) + t1*((j&2)>>1)) * scale;
            Float cost = (o0t-o1t).squaredNorm();

            if (cost < best_cost) {
                best_i = i;
                best_j = j;
                best_cost = cost;
            }
        }
    }

	compat1 = o0 + (q0*(best_i & 1) + t0*((best_i & 2) >> 1)) * scale;
	compat2 = o1 + (q1*(best_j & 1) + t1*((best_j & 2) >> 1)) * scale;
}

template <>
std::pair<Vector2i, Vector2i> PositionFieldTraits<6>::findCompatibleIndex(
        const Vector3f &p0, const Vector3f &n0, const Vector3f &q0, const Vector3f &o0,
        const Vector3f &p1, const Vector3f &n1, const Vector3f &q1, const Vector3f &o1,
        Float scale, Float inv_scale, Float* error) {
    Vector3f t0 = rotate60(q0, n0), t1 = rotate60(q1, n1);
    Vector3f middle = middle_point(p0, n0, p1, n1);
    Vector2i o0i = position_floor_index_3(o0, q0, n0, middle, scale, inv_scale);
    Vector2i o1i = position_floor_index_3(o1, q1, n1, middle, scale, inv_scale);

    Float best_cost = std::numeric_limits<Float>::infinity();
    int best_i = -1, best_j = -1;
    for (int i=0; i<4; ++i) {
        Vector3f o0t = o0 + (q0*(o0i.x() + (i&1)) + t0*(o0i.y() + ((i&2)>>1))) * scale;
        for (int j=0; j<4; ++j) {
            Vector3f o1t = o1 + (q1*(o1i.x() + (j&1)) + t1*(o1i.y() + ((j&2)>>1))) * scale;
            Float cost = (o0t-o1t).squaredNorm();

            if (cost < best_cost) {
                best_i = i;
                best_j = j;
                best_cost = cost;
            }
        }
    }

	if (error)
		*error = best_cost;

    return std::make_pair(
        Vector2i(o0i.x()+(best_i&1), o0i.y()+((best_i&2)>>1)),
        Vector2i(o1i.x()+(best_j&1), o1i.y()+((best_j&2)>>1)));
}
