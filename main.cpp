#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>
#include <algorithm>

#include <string>
#include <stdio.h>

#include <list>

#include <random>
static std::default_random_engine engine(10); // random seed = 10
static std::uniform_real_distribution<double> uniform(0, 1);

#define M_Pi 3.14159265358979323846

class Vector {
public:
	explicit Vector(double x = 0, double y = 0, double z = 0) {
		coords[0] = x;
		coords[1] = y;
		coords[2] = z;
	};
	double operator[] (int i) const { return coords[i]; };
	double& operator[] (int i) { return coords[i]; };
	double sqrNorm() const {
		return coords[0] * coords[0] + coords[1] * coords[1] + coords[2] * coords[2];
	};

	Vector& operator+=(const Vector& a) {
		coords[0] += a[0];
		coords[1] += a[1];
		coords[2] += a[2];
		return *this;
	};
	Vector normalize() {
		double norm = sqrt(sqrNorm());
		return Vector(coords[0] / norm, coords[1] / norm, coords[2] / norm);
	};
private:
	double coords[3];
};

// Définitions des opérations possibles sur les vecteurs

Vector operator+(const Vector& a, const Vector& b) {
	return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}

Vector operator-(const Vector& a, const Vector& b) {
	return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

Vector operator-(const Vector& a) {
	return Vector(-a[0], -a[1], -a[2]);
}

Vector operator*(double a, const Vector& b) {
	return Vector(a * b[0], a * b[1], a * b[2]);
}

Vector operator*(const Vector& a, double b) {
	return Vector(a[0] * b, a[1] * b, a[2] * b);
}

Vector operator*(const Vector& a, const Vector& b) {
	return Vector(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
}

Vector operator/(const Vector& a, double b) {
	return Vector(a[0] / b, a[1] / b, a[2] / b);
}

Vector cross(const Vector& a, const Vector& b) {
	return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

double dot(const Vector& a, const Vector& b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double sqr(const double& a) {
	return a * a;
}

Vector random_cos(const Vector& N) {
	double r1 = uniform(engine);
	double r2 = uniform(engine);
	double x = sqrt(1 - r2) * cos(2 * M_Pi * r1);
	double y = sqrt(1 - r2) * sin(2 * M_Pi * r1);
	double z = sqrt(r2);
	Vector T1;

	/* Pour T1, on trouve la plus petite composante et on la fixe à 0
	On inverse ensuite les deux autres composantes en prenant l'opposé pour une des deux */

	if (N[0] < N[1] && N[0] < N[2]) {
		T1 = Vector(0, -N[2], N[1]);
	}
	else if (N[1] < N[0] && N[1] < N[2]) {
		T1 = Vector(N[2], 0, -N[0]);
	}
	else {
		T1 = Vector(-N[1], N[0], 0);
	}
	T1.normalize();
	Vector T2 = cross(N, T1).normalize();
	return (x * T1 + y * T2 + z * N).normalize();
}

class Ray {
public:
	Ray(const Vector& origin, const Vector& dir) : origin(origin), dir(dir) {
	}
	Vector origin, dir;
};

class Object {
public:
	Object() {};

	virtual bool intersect(const Ray& r, Vector& P, Vector& normal, double& t, Vector& color) = 0;

	Vector albedo;
	double n_refract;
	bool isMirror;
	bool isTransp;
};

class Sphere : public Object {
public:
	Sphere(const Vector& O, double R, const Vector albedo, double n_refract = 1., bool isMirror = false, bool isTransp = false) : O(O), R(R) {
		this->albedo = albedo;
		this->n_refract = n_refract;
		this->isMirror = isMirror;
		this->isTransp = isTransp;
	}
	bool intersect(const Ray& r, Vector& P, Vector& N, double& t, Vector& color) {
		// Termes de l'équation du second degré
		double a = 1;
		double b = 2 * dot(r.dir, r.origin - O);
		double c = (r.origin - O).sqrNorm() - R * R;

		// Calcul du discriminant
		double delta = b * b - 4 * a * c;

		if (delta < 0) {
			return false;
		}

		double sqrtDelta = sqrt(delta);
		// Définition de la plus grande racine
		double t2 = (-b + sqrtDelta) / (2 * a);
		if (t2 < 0) return false;

		double t1 = (-b - sqrtDelta) / (2 * a);
		// On récupère la plus faible racine
		if (t1 > 0) {
			t = t1;
		}
		else {
			t = t2;
		}

		P = r.origin + t * r.dir;
		N = (P - O).normalize();
		color = this->albedo;
		return true;
	};
	Vector O;
	double R;
};

class BoundingBox {
public:
	Vector mini, maxi;
	bool intersect(const Ray& r) {
		double t1x = (mini[0] - r.origin[0]) / r.dir[0];
		double t2x = (maxi[0] - r.origin[0]) / r.dir[0];
		double txMin = std::min(t1x, t2x);
		double txMax = std::max(t1x, t2x);

		double t1y = (mini[1] - r.origin[1]) / r.dir[1];
		double t2y = (maxi[1] - r.origin[1]) / r.dir[1];
		double tyMin = std::min(t1y, t2y);
		double tyMax = std::max(t1y, t2y);

		double t1z = (mini[2] - r.origin[2]) / r.dir[2];
		double t2z = (maxi[2] - r.origin[2]) / r.dir[2];
		double tzMin = std::min(t1z, t2z);
		double tzMax = std::max(t1z, t2z);

		double tMax = std::min(txMax, std::min(tyMax, tzMax));
		double tMin = std::max(txMin, std::max(tyMin, tzMin));

		if (tMax < 0) {
			return false;
		}
		return tMax > tMin;
	}
};

class Noeud {
public:
	Noeud* fg;
	Noeud* fd;
	BoundingBox Bbox;
	int begin, end;
};

class Scene {
public:
	Scene(const Vector& Lum, double I) : Lum(Lum), I(I) {
	}
	bool intersect(const Ray& r, Vector& P, Vector& N, Vector& albedo, double& n_refract, bool& isMirror, bool& isTransp, double& t, int& objectID) {
		t = 1E10;
		bool has_inter = false;
		for (int i = 0; i < objects.size(); i++) {
			Vector localP, localN, localAlbedo;
			double localt;
			if ((objects[i]->intersect(r, localP, localN, localt, localAlbedo)) && (localt < t)) {
				has_inter = true;
				P = localP;
				N = localN;
				t = localt;
				albedo = localAlbedo;
				n_refract = objects[i]->n_refract;
				isMirror = objects[i]->isMirror;
				isTransp = objects[i]->isTransp;
				objectID = i;
			}
		}
		return has_inter;
	}

	Vector getColor(const Ray& r, const int& rebound) {

		if (rebound > 5) return Vector(0., 0., 0.);

		Vector P, N, albedo;
		double t, n_refract;
		bool isMirror, isTransp;
		int objectID;
		bool inter = intersect(r, P, N, albedo, n_refract, isMirror, isTransp, t, objectID);

		Vector color(0., 0., 0.);
		if (inter) {

			double eps = 0.001;

			if (objectID == 0) {
				if (rebound == 0) {
					return Vector(I, I, I) / (4 * M_Pi * M_Pi * sqr(dynamic_cast<Sphere*>(objects[0])->R));
				}
				else {
					return Vector(0., 0., 0.);
				}
			}

			if (isMirror) {  // Gestion des sphères "miroir"
				Vector reflectDir = r.dir - 2 * dot(r.dir, N) * N;
				Ray reflectRay(P + eps * N, reflectDir.normalize());
				return getColor(reflectRay, rebound + 1);
			}
			else {

				if (isTransp) {  // Gestion des sphères transparentes

					double n1 = 1;
					double n2 = n_refract;
					Vector N_transp = N;

					// Si le rayon sort de la sphère, on inverse la normale et les indices de réfraction
					if (dot(r.dir, N) > 0) {  
						std::swap(n1, n2);
						N_transp = -N;
					}

					Vector T_t = n1 / n2 * (r.dir - dot(r.dir, N_transp) * N_transp);
					double radical = 1 - sqr(n1 / n2) * (1 - sqr(dot(r.dir, N_transp)));

					if (radical < 0) {   // Racine complexe --> réflexion totale 
						Vector reflectDir = r.dir - 2 * dot(r.dir, N) * N;
						Ray reflectRay(P + eps * N, reflectDir.normalize());
						return getColor(reflectRay, rebound + 1);
					}

					Vector T_n = -sqrt(radical) * N_transp;
					Vector refractDir = T_t + T_n;
					return getColor(Ray(P - eps * N_transp, refractDir.normalize()), rebound + 1);
				}

				// Cas général
				else {   // Partie 1 : Eclairage direct - Version 1
					/*
					Vector PL = Lum - P;
					double dist = sqrt(PL.sqrNorm());

					// Gestion des ombres portées
					Vector shadowP, shadowN, shadowAlbedo;
					double shadowt, shadowIndiceRefr;
					bool shadowIsMirror, shadowIsTransp;
					int shadowObjectID;
					Ray shadowRay(P + N * eps, PL / dist);
					bool shadowInter = intersect(shadowRay, shadowP, shadowN, shadowAlbedo, shadowIndiceRefr, shadowIsMirror, shadowIsTransp, shadowt, shadowObjectID);

					if (shadowInter && (shadowt < dist)) {
						color = Vector(0., 0., 0.);
					}
					else {  // Couleur "de base"
						color = I / (4. * M_Pi * dist * dist) * albedo / M_Pi * std::max(0., dot(N, PL / dist));
					} */


					// Eclairage direct - Version 2
					Vector PL = Lum - P;
					Vector random_dir = random_cos(-PL.normalize());
					Vector position = random_dir * dynamic_cast<Sphere*>(objects[0])->R + dynamic_cast<Sphere*>(objects[0])->O;
					Vector PL_prime = position - P;
					double dist_prime = sqrt(PL_prime.sqrNorm());
					PL_prime = PL_prime.normalize();

					Vector shadowP, shadowN, shadowAlbedo;
					double shadowt, shadowIndiceRefr;
					bool shadowIsMirror, shadowIsTransp;
					int shadowObjectID;
					Ray shadowRay(P + N * eps, PL_prime);
					bool shadowInter = intersect(shadowRay, shadowP, shadowN, shadowAlbedo, shadowIndiceRefr, shadowIsMirror, shadowIsTransp, shadowt, shadowObjectID);

					if (shadowInter && shadowt < dist_prime - 0.005) {
						color = Vector(0., 0., 0.);
					}
					else {
						double proba = std::max(0., dot(-PL, random_dir)) / (M_Pi * dynamic_cast<Sphere*>(objects[0])->R * dynamic_cast<Sphere*>(objects[0])->R);
						double jacob = std::max(0., dot(random_dir, -PL_prime)) / (dist_prime * dist_prime);
						color = I / (4. * M_Pi * M_Pi * dynamic_cast<Sphere*>(objects[0])->R * dynamic_cast<Sphere*>(objects[0])->R) * albedo / M_Pi * std::max(0., dot(N, PL_prime)) * jacob / proba;
					}

					// Partie 2 : Eclairage indirect
					Vector wi = random_cos(N);
					Ray wiRay(P + N * 0.0001, wi);
					color += albedo * getColor(wiRay, rebound + 1);
				}
			}
		}
		return color;
	}
	std::vector<Object*> objects;
	Vector Lum;
	double I;
};

class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
	};
	int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
	int uvi, uvj, uvk;  // indices within the uv coordinates array
	int ni, nj, nk;  // indices within the normals array
	int group;       // face group
};


class TriangleMesh : public Object {
public:
	~TriangleMesh() {}
	TriangleMesh(const Vector& albedo, double n_refract = 1., bool mirror = false, bool transp = false) {
		this->albedo = albedo;
		this->n_refract = n_refract;
		this->isMirror = mirror;
		this->isTransp = transp;
		BVH = new Noeud;
	};

	void readOBJ(const char* obj) {

		char matfile[255];
		char grp[255];

		FILE* f;
		f = fopen(obj, "r");
		int curGroup = -1;
		while (!feof(f)) {
			char line[255];
			if (!fgets(line, 255, f)) break;

			std::string linetrim(line);
			linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
			strcpy(line, linetrim.c_str());

			if (line[0] == 'u' && line[1] == 's') {
				sscanf(line, "usemtl %[^\n]\n", grp);
				curGroup++;
			}

			if (line[0] == 'v' && line[1] == ' ') {
				Vector vec;

				Vector col;
				if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
					col[0] = std::min(1., std::max(0., col[0]));
					col[1] = std::min(1., std::max(0., col[1]));
					col[2] = std::min(1., std::max(0., col[2]));

					vertices.push_back(vec);
					vertexcolors.push_back(col);

				}
				else {
					sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
					vertices.push_back(vec);
				}
			}
			if (line[0] == 'v' && line[1] == 'n') {
				Vector vec;
				sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
				normals.push_back(vec);
			}
			if (line[0] == 'v' && line[1] == 't') {
				Vector vec;
				sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
				uvs.push_back(vec);
			}
			if (line[0] == 'f') {
				TriangleIndices t;
				int i0, i1, i2, i3;
				int j0, j1, j2, j3;
				int k0, k1, k2, k3;
				int nn;
				t.group = curGroup;

				char* consumedline = line + 1;
				int offset;

				nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
				if (nn == 9) {
					if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
					if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
					if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
					if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
					if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
					if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
					if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
					if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
					if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
					indices.push_back(t);
				}
				else {
					nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
					if (nn == 6) {
						if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
						if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
						if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
						if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
						if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
						if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
						indices.push_back(t);
					}
					else {
						nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
						if (nn == 3) {
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							indices.push_back(t);
						}
						else {
							nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
							if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
							if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
							if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
							if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
							if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
							if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
							indices.push_back(t);
						}
					}
				}

				consumedline = consumedline + offset;

				while (true) {
					if (consumedline[0] == '\n') break;
					if (consumedline[0] == '\0') break;
					nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
					TriangleIndices t2;
					t2.group = curGroup;
					if (nn == 3) {
						if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
						if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
						if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
						if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
						if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
						if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
						if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
						if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
						if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
						indices.push_back(t2);
						consumedline = consumedline + offset;
						i2 = i3;
						j2 = j3;
						k2 = k3;
					}
					else {
						nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
						if (nn == 2) {
							if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
							if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
							if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
							if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
							if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
							if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
							consumedline = consumedline + offset;
							i2 = i3;
							j2 = j3;
							indices.push_back(t2);
						}
						else {
							nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
							if (nn == 2) {
								if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
								if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
								if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
								if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
								if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
								if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
								consumedline = consumedline + offset;
								i2 = i3;
								k2 = k3;
								indices.push_back(t2);
							}
							else {
								nn = sscanf(consumedline, "%u%n", &i3, &offset);
								if (nn == 1) {
									if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
									if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
									if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
									consumedline = consumedline + offset;
									i2 = i3;
									indices.push_back(t2);
								}
								else {
									consumedline = consumedline + 1;
								}
							}
						}
					}
				}

			}

		}
		fclose(f);
	}

	BoundingBox buildBB(int begin, int end) {

		BoundingBox bb;
		bb.mini = Vector(1E9, 1E9, 1E9);
		bb.maxi = Vector(-1E9, -1E9, -1E9);
		for (int i = begin; i < end; i++) {
			for (int j = 0; j < 3; j++) {
				bb.mini[j] = std::min(bb.mini[j], vertices[indices[i].vtxi][j]);
				bb.maxi[j] = std::max(bb.maxi[j], vertices[indices[i].vtxi][j]);
				bb.mini[j] = std::min(bb.mini[j], vertices[indices[i].vtxj][j]);
				bb.maxi[j] = std::max(bb.maxi[j], vertices[indices[i].vtxj][j]);
				bb.mini[j] = std::min(bb.mini[j], vertices[indices[i].vtxk][j]);
				bb.maxi[j] = std::max(bb.maxi[j], vertices[indices[i].vtxk][j]);
			}
		}
		return bb;
	};

	void buildBVH(Noeud* n, int begin, int end) {

		// Initialisation
		n->begin = begin;
		n->end = end;
		n->Bbox = buildBB(n->begin, n->end);
		n->fg = NULL;
		n->fd = NULL;

		// On détermine la direction dans laquelle scinder la BoundingBox
		Vector diag = n->Bbox.maxi - n->Bbox.mini;
		int dimension;
		if ((diag[0] >= diag[1]) && (diag[0] >= diag[2])) {
			dimension = 0;
		}
		else if ((diag[1] >= diag[0]) && (diag[1] >= diag[2])) {
			dimension = 1;
		}
		else {
			dimension = 2;
		}
		double milieu = (n->Bbox.mini[dimension] + n->Bbox.maxi[dimension]) / 2;

		// 
		int ind_pivot = n->begin;
		for (int i = n->begin; i < n->end; i++) {
			double centre_triangle = (vertices[indices[i].vtxi][dimension] + vertices[indices[i].vtxj][dimension] + vertices[indices[i].vtxk][dimension]) / 3;
			if (centre_triangle < milieu) {
				std::swap(indices[i], indices[ind_pivot]);
				ind_pivot++;
			}
		}

		// Critère d'arrêt
		if (ind_pivot == begin || ind_pivot == end) {
			return;
		}
		if (end - begin < 5) {
			return;
		}

		n->fg = new Noeud();
		n->fd = new Noeud();

		// Appel récursif
		buildBVH(n->fg, n->begin, ind_pivot);
		buildBVH(n->fd, ind_pivot, n->end);
	};

	bool intersect(const Ray& r, Vector& P, Vector& normal, double& t, Vector& color) {
		if (!BVH->Bbox.intersect(r)) return false;

		t = 1E10;
		bool has_inter = false;

		std::list<Noeud*> liste;
		liste.push_back(BVH);

		while (!liste.empty()) {

			Noeud* current = liste.front();
			liste.pop_front();

			if ((current->fg) && (current->fg->Bbox.intersect(r))) {
				liste.push_front(current->fg);
			}

			if ((current->fd) && (current->fd->Bbox.intersect(r))) {
				liste.push_front(current->fd);
			}

			if (!current->fg) {

				for (int i = current->begin; i < current->end; i++) {
					const Vector& A = vertices[indices[i].vtxi];
					const Vector& B = vertices[indices[i].vtxj];
					const Vector& C = vertices[indices[i].vtxk];

					Vector oppC = B - A;
					Vector oppB = C - A;
					Vector Nbis = cross(oppC, oppB);

					Vector AO = r.origin - A;
					Vector AOu = cross(AO, r.dir);
					double UN = dot(r.dir, Nbis);
					double beta = -dot(oppB, AOu) / UN;
					double gamma = dot(oppC, AOu) / UN;

					double localt = -dot(AO, Nbis) / UN;

					double alpha = 1 - beta - gamma;

					if ((beta >= 0) && (gamma >= 0) && (beta <= 1) && (gamma <= 1) && (alpha >= 0) && (localt > 0)) {
						has_inter = true;
						if (localt < t) {
							t = localt;
							normal = alpha * normals[indices[i].ni] + beta * normals[indices[i].nj] + gamma * normals[indices[i].nk];
							normal.normalize();
							P = r.origin + t * r.dir;

							// Gestion des textures
							Vector UV = alpha * uvs[indices[i].uvi] + beta * uvs[indices[i].uvj] + gamma * uvs[indices[i].uvk];
							int H = Htex[indices[i].group];
							int W = Wtex[indices[i].group];
							UV = UV * Vector(Wtex[indices[i].group], Htex[indices[i].group], 0);
							int UVx = UV[0] + 0.5;
							int UVy = UV[1] + 0.5;
							UVx = UVx % W;
							UVy = UVy % H;
							if (UVx < 0) { UVx += W; }
							if (UVy < 0) { UVx += H; }

							// Inversion de la texture
							UVy = H - UVy - 1;
							color = Vector(std::pow(textures[indices[i].group][(UVy * H + UVx) * 3] / 255., 2.2),
								std::pow(textures[indices[i].group][(UVy * H + UVx) * 3 + 1] / 255., 2.2),
								std::pow(textures[indices[i].group][(UVy * H + UVx) * 3 + 2] / 255., 2.2));
						}
					}
				}
			}
		}
		return has_inter;
	}

	void loadTexture(const char* filename) {
		int W, H, C;
		unsigned char* texture = stbi_load(filename, &W, &H, &C, 3);
		Wtex.push_back(W);
		Htex.push_back(H);
		textures.push_back(texture);
	}

	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;
	std::vector<unsigned char*> textures;
	std::vector<int> Wtex, Htex;
	BoundingBox Bbox;
	Noeud* BVH;
};

int main() {
	int W = 256;
	int H = 256;
	int nbrays = 100;

	double fov = M_Pi / 3;

	double gamma = (1. / 2.2);

	double I = 1.3E11;
	Vector rho(1, 1, 1);
	Vector Lum(-10, 20, 40);

	Vector C(0, 0, 55);
	Vector O(0, 0, 0);
	double R = 10;

	Scene scene(Lum, I);

	Sphere Slum(Lum, 5, Vector(1., 1., 1.));

	TriangleMesh Dog(Vector(1., 1., 1.));
	Dog.readOBJ("13463_Australian_Cattle_Dog_v3.obj");
	Dog.loadTexture("Australian_Cattle_Dog_dif.jpg");

	
	for (int i = 0; i < Dog.vertices.size(); i++) {
		Dog.vertices[i][1] += 10;
		Dog.vertices[i][1] = -Dog.vertices[i][1];

		std::swap(Dog.vertices[i][1], Dog.vertices[i][2]);
		Dog.vertices[i][1] -= 10;
	}
	for (int i = 0; i < Dog.normals.size(); i++) {
		std::swap(Dog.normals[i][1], Dog.normals[i][2]);
		Dog.normals[i][2] = -Dog.normals[i][2];
	}

	Sphere S1(O, R, Vector(1., 1., 1.));
	Sphere S2(Vector(10, 0, 20), R, Vector(1., 1., 1.));
	Sphere S3(Vector(-10, 0, -20), R, Vector(1., 1., 1.));

	Sphere Ssol(Vector(0., -1000., 0.), 990., Vector(1., 1., 1.));
	Sphere Smur1(Vector(-1000., 0., 0.), 940., Vector(1., 0.5, 0.5));
	Sphere Smur2(Vector(1000., 0., 0.), 940., Vector(0.5, 1., 0.5));
	Sphere Smur3(Vector(0., 0., -1000.), 940., Vector(0.2, 1., 1.));
	Sphere Smur4(Vector(0., 0., 1000.), 940., Vector(1., 1., 0.5));
	Sphere Splaf(Vector(0., 1000., 0.), 940., Vector(1., 1., 1.));

	// Dog.buildBB();

	Dog.buildBVH(Dog.BVH, 0, Dog.indices.size());

	scene.objects.push_back(&Slum);
	// scene.objects.push_back(&S1);
	// scene.objects.push_back(S2);
	// scene.objects.push_back(S3);
	scene.objects.push_back(&Ssol);
	scene.objects.push_back(&Smur1);
	scene.objects.push_back(&Smur2);
	scene.objects.push_back(&Smur3);
	scene.objects.push_back(&Smur4);
	scene.objects.push_back(&Splaf);
	scene.objects.push_back(&Dog);

	double horizontal = 0;
	double vertical = 0;
	Vector up(0, cos(horizontal), sin(horizontal));
	Vector right(cos(vertical), 0, sin(vertical));
	Vector viewDirection = -cross(up, right);

	std::vector<unsigned char> image(W * H * 3, 0);
#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {

			Vector color(0., 0., 0.);

			for (int k = 0; k < nbrays; k++) {
				double u1 = uniform(engine);
				double u2 = uniform(engine);
				double x = 0.25 * cos(2 * M_Pi * u1) * sqrt(-2 * log(u2));
				double y = 0.25 * sin(2 * M_Pi * u1) * sqrt(-2 * log(u2));

				double u3 = uniform(engine);
				double u4 = uniform(engine);
				double x3 = 0.25 * cos(2 * M_Pi * u3) * sqrt(-2 * log(u4));
				double x4 = 0.25 * sin(2 * M_Pi * u3) * sqrt(-2 * log(u4));

				Vector u(j - W / 2. + x + 0.5, i - H / 2. + y + 0.5, -W / (2 * tan(fov / 2)));
				u = u[0] * right + u[1] * up + u[2] * viewDirection;
				u = u.normalize();

				Vector target = C + 55 * u;
				Vector new_origin = C + Vector(x3, x4, 0);
				Vector new_dir = (target - new_origin).normalize();
				Ray r(new_origin, new_dir);

				color += scene.getColor(r, int(0));
			}
			color = color / nbrays;

			image[((H - i - 1) * W + j) * 3 + 0] = std::min(255., std::pow(color[0], gamma));
			image[((H - i - 1) * W + j) * 3 + 1] = std::min(255., std::pow(color[1], gamma));
			image[((H - i - 1) * W + j) * 3 + 2] = std::min(255., std::pow(color[2], gamma));
		}
	}
	stbi_write_png("image-finale.png", W, H, 3, &image[0], 0);

	return 0;
}