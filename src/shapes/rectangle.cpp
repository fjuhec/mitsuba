/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <mitsuba/render/shape.h>
#include <mitsuba/render/bsdf.h>
#include <mitsuba/render/emitter.h>
#include <mitsuba/render/sensor.h>
#include <mitsuba/render/subsurface.h>
#include <mitsuba/render/medium.h>
#include <mitsuba/render/trimesh.h>
#include <mitsuba/core/properties.h>

MTS_NAMESPACE_BEGIN

/*!\plugin{rectangle}{Rectangle intersection primitive}
 * \order{3}
 * \parameters{
 *     \parameter{toWorld}{\Transform\Or\Animation}{
 *	      Specifies a linear object-to-world transformation.
 *        It is allowed to use non-uniform scaling, but no shear.
 *        \default{none (i.e. object space $=$ world space)}
 *     }
 *     \parameter{flipNormals}{\Boolean}{
 *	      Is the rectangle inverted, i.e. should the normal vectors
 *		  be flipped? \default{\code{false}}
 *	   }
 * }
 * \renderings{
 *     \rendering{Two rectangles configured as a reflective surface and an
 *     emitter (\lstref{rectangle})}{shape_rectangle}
 * }
 *
 * This shape plugin describes a simple rectangular intersection primitive.
 * It is mainly provided as a convenience for those cases when creating and
 * loading an external mesh with two triangles is simply too tedious, e.g.
 * when an area light source or a simple ground plane are needed.
 *
 * By default, the rectangle covers the XY-range $[-1,1]\times[-1,1]$
 * and has a surface normal that points into the positive $Z$ direction.
 * To change the rectangle scale, rotation, or translation, use the
 * \code{toWorld} parameter.
 *
 * \vspace{2mm}
 * \begin{xml}[caption={A simple example involving two rectangle instances}, label=lst:rectangle]
 * <scene version=$\MtsVer$>
 *     <shape type="rectangle">
 *         <bsdf type="diffuse"/>
 *     </shape>
 *     <shape type="rectangle">
 *         <transform name="toWorld">
 *             <rotate x="1" angle="90"/>
 *             <scale x="0.4" y="0.3" z="0.2"/>
 *             <translate y="1" z="0.2"/>
 *         </transform>
 *         <emitter type="area">
 *             <spectrum name="intensity" value="3"/>
 *         </emitter>
 *     </shape>
 *     <!-- ... other definitions ... -->
 * </scene>
 * \end{xml}
 */
class Rectangle : public Shape {
public:
	Rectangle(const Properties &props) : Shape(props) {
		m_objectToWorld = new AnimatedTransform(props.getAnimatedTransform("toWorld", Transform()));

		if (props.getBoolean("flipNormals", false))
			m_objectToWorld->prependScale(Vector(1, 1, -1));
	}

	Rectangle(Stream *stream, InstanceManager *manager)
			: Shape(stream, manager) {
		m_objectToWorld = new AnimatedTransform(stream);
		configure();
	}

	void serialize(Stream *stream, InstanceManager *manager) const {
		Shape::serialize(stream, manager);
		m_objectToWorld->serialize(stream);
	}

	void configure() {
		Shape::configure();

		const Transform &trafo = m_objectToWorld->eval(0);
		Vector dpdu = trafo(Vector(2, 0, 0));
		Vector dpdv = trafo(Vector(0, 2, 0));

		m_invSurfaceArea = 1.0f / getSurfaceArea();
		if (std::abs(dot(normalize(dpdu), normalize(dpdv))) > Epsilon)
			Log(EError, "Error: 'toWorld' transformation contains shear!");
	}

	AABB getAABB() const {
		std::set<Float> times;
		m_objectToWorld->collectKeyframes(times);

		AABB aabb;
		for (std::set<Float>::iterator it = times.begin(); it != times.end(); ++it) {
			const Transform &trafo = m_objectToWorld->eval(*it);
			aabb.expandBy(trafo(Point(-1, -1, 0)));
			aabb.expandBy(trafo(Point( 1, -1, 0)));
			aabb.expandBy(trafo(Point( 1,  1, 0)));
			aabb.expandBy(trafo(Point(-1,  1, 0)));
		}
		return aabb;
	}

	Float getSurfaceArea() const {
		const Transform &trafo = m_objectToWorld->eval(0);
		Vector dpdu = trafo(Vector(2, 0, 0));
		Vector dpdv = trafo(Vector(0, 2, 0));
		return dpdu.length() * dpdv.length();
	}

	inline bool rayIntersect(const Ray &_ray, Float mint, Float maxt, Float &t, void *temp) const {
		Ray ray;
		m_objectToWorld->eval(_ray.time).inverse().transformAffine(_ray, ray);
		Float hit = -ray.o.z / ray.d.z;

		if (!(hit >= mint && hit <= maxt))
			return false;

		Point local = ray(hit);

		if (std::abs(local.x) <= 1 && std::abs(local.y) <= 1) {
			t = hit;

			if (temp) {
				Float *data = static_cast<Float *>(temp);
				data[0] = local.x;
				data[1] = local.y;
			}

			return true;
		}

		return false;
	}

	bool rayIntersect(const Ray &ray, Float mint, Float maxt) const {
		Float t;
		return Rectangle::rayIntersect(ray, mint, maxt, t, NULL);
	}

	void fillIntersectionRecord(const Ray &ray,
			const void *temp, Intersection &its) const {
		const Float *data = static_cast<const Float *>(temp);
		const Transform &trafo = m_objectToWorld->eval(ray.time);
		Vector dpdu = trafo(Vector(2, 0, 0));
		Vector dpdv = trafo(Vector(0, 2, 0));
		Normal normal = normalize(trafo(Normal(0, 0, 1)));
		Frame frame = Frame(normalize(dpdu), normalize(dpdv), normal);
		its.geoFrame = frame;
		its.shFrame = its.geoFrame.n;
		its.shape = this;
		its.dpdu = dpdu;
		its.dpdv = dpdv;
		its.uv = Point2(0.5f * (data[0]+1), 0.5f * (data[1]+1));
		its.p = ray(its.t);
 		its.hasUVPartials = false;
		its.instance = NULL;
		its.time = ray.time;
	}

	ref<TriMesh> createTriMesh() {
		ref<TriMesh> mesh = new TriMesh(getName(),
			2, 4, true, true, false);

		Point *vertices = mesh->getVertexPositions();
		Normal *normals = mesh->getVertexNormals();
		Point2 *texcoords = mesh->getVertexTexcoords();
		Triangle *triangles = mesh->getTriangles();

		const Transform &trafo = m_objectToWorld->eval(0.0f);
		vertices[0] = trafo(Point(-1, -1, 0));
		vertices[1] = trafo(Point( 1, -1, 0));
		vertices[2] = trafo(Point( 1,  1, 0));
		vertices[3] = trafo(Point(-1,  1, 0));

		texcoords[0] = Point2(0, 0);
		texcoords[1] = Point2(1, 0);
		texcoords[2] = Point2(1, 1);
		texcoords[3] = Point2(0, 1);

		normals[0] = normals[1] = normals[2] = normals[3] = normalize(trafo(Normal(0, 0, 1)));
		triangles[0].idx[0] = 0;
		triangles[0].idx[1] = 1;
		triangles[0].idx[2] = 2;

		triangles[1].idx[0] = 2;
		triangles[1].idx[1] = 3;
		triangles[1].idx[2] = 0;

		mesh->setBSDF(m_bsdf);
		mesh->setEmitter(m_emitter);
		mesh->configure();

		return mesh.get();
	}

	void getNormalDerivative(const Intersection &its,
			Vector &dndu, Vector &dndv, bool shadingFrame) const {
		dndu = dndv = Vector(0.0f);
	}

	void samplePosition(PositionSamplingRecord &pRec, const Point2 &sample) const {
		const Transform &trafo = m_objectToWorld->eval(pRec.time);
		pRec.p = trafo(Point3(sample.x * 2 - 1, sample.y * 2 - 1, 0));
		pRec.n = normalize(trafo(Normal(0, 0, 1)));
		pRec.pdf = m_invSurfaceArea;
		pRec.measure = EArea;
		pRec.uv = sample;
	}

	Float pdfPosition(const PositionSamplingRecord &pRec) const {
		return m_invSurfaceArea;
	}

	size_t getPrimitiveCount() const {
		return 1;
	}

	size_t getEffectivePrimitiveCount() const {
		return 1;
	}

	std::string toString() const {
		std::ostringstream oss;
		oss << "Rectangle[" << endl
			<< "  objectToWorld = " << indent(m_objectToWorld.toString()) << "," << endl;
		if (isMediumTransition())
			oss << "  interiorMedium = " << indent(m_interiorMedium.toString()) << "," << endl
				<< "  exteriorMedium = " << indent(m_exteriorMedium.toString()) << "," << endl;
		oss << "  bsdf = " << indent(m_bsdf.toString()) << "," << endl
			<< "  emitter = " << indent(m_emitter.toString()) << "," << endl
			<< "  sensor = " << indent(m_sensor.toString()) << "," << endl
			<< "  subsurface = " << indent(m_subsurface.toString()) << endl
			<< "]";
		return oss.str();
	}

	MTS_DECLARE_CLASS()
private:
	ref<AnimatedTransform> m_objectToWorld;
	Float m_invSurfaceArea;
};

MTS_IMPLEMENT_CLASS_S(Rectangle, false, Shape)
MTS_EXPORT_PLUGIN(Rectangle, "Rectangle intersection primitive");
MTS_NAMESPACE_END
