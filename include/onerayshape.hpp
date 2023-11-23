#pragma once

#include <string>
#include "gazebo/physics/RayShape.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/util/system.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODEPhysics.hh"
#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/physics/ode/ODECollision.hh"
#include "gazebo/physics/ode/ODERayShape.hh"

namespace gazebo
{
namespace physics
{

/// \brief Ray collision
class OneRayShape : public RayShape
{
public:
    explicit OneRayShape(CollisionPtr _parent) : RayShape(_parent)
    {    
        this->SetName("ODE Ray Shape");

        ODECollisionPtr collision =
        boost::static_pointer_cast<ODECollision>(this->collisionParent);

        this->physicsEngine = boost::static_pointer_cast<ODEPhysics>(
        this->collisionParent->GetWorld()->Physics());

        collision->SetSpaceId(dSimpleSpaceCreate(0));
        this->geomId = dCreateRay(collision->GetSpaceId(), 1.0);

        // Create default ray with unit length
        collision->SetCollision(this->geomId, false);
        collision->SetCategoryBits(GZ_SENSOR_COLLIDE);
        collision->SetCollideBits(~GZ_SENSOR_COLLIDE);

    }

    virtual ~OneRayShape()
    {
        dGeomDestroy(this->geomId);
    }

    void GetIntersection(double &_dist, std::string &_entity)
    {
        if (this->physicsEngine)
        {
            Intersection intersection;
            intersection.depth = 1000;

            {
                boost::recursive_mutex::scoped_lock lock(
                    *this->physicsEngine->GetPhysicsUpdateMutex());

                // Do collision detection
                dSpaceCollide2(this->geomId,
                    (dGeomID)(this->physicsEngine->GetSpaceId()),
                    &intersection, &UpdateCallback);
            }
            _dist = intersection.depth;
            _entity = intersection.name;
        }
    }

    void SetPoints(const ignition::math::Vector3d &_posStart, const ignition::math::Vector3d &_posEnd)
    {
        this->globalStartPos = _posStart;
        this->globalEndPos = _posEnd;

        dir = this->globalEndPos - this->globalStartPos;
        dir.Normalize();

        if (!ignition::math::equal(this->contactLen, 0.0))
        {
            dGeomRaySet(this->geomId,
                this->globalStartPos.X(),
                this->globalStartPos.Y(),
                this->globalStartPos.Z(),
                dir.X(), dir.Y(), dir.Z());

            dGeomRaySetLength(this->geomId,
                this->globalStartPos.Distance(this->globalEndPos));
        }
    }

    virtual void Update() override {}

    ignition::math::Vector3d getDir() const { return dir; }

private:

    static void UpdateCallback(void *_data, dGeomID _o1, dGeomID _o2)
    {
        dContactGeom contact;
        OneRayShape::Intersection *inter = nullptr;

        inter = static_cast<Intersection*>(_data);

        // Check space
        if (dGeomIsSpace(_o1) || dGeomIsSpace(_o2))
        {
            dSpaceCollide2(_o1, _o2, inter, &UpdateCallback);
        }
        else
        {
            ODECollision *collision1, *collision2;
            ODECollision *hitCollision = nullptr;

            // Get pointers to the underlying collisions
            if (dGeomGetClass(_o1) == dGeomTransformClass)
                collision1 = static_cast<ODECollision*>(dGeomGetData(dGeomTransformGetGeom(_o1)));
            else
                collision1 = static_cast<ODECollision*>(dGeomGetData(_o1));

            if (dGeomGetClass(_o2) == dGeomTransformClass)
                collision2 = static_cast<ODECollision*>(dGeomGetData(dGeomTransformGetGeom(_o2)));
            else
                collision2 = static_cast<ODECollision*>(dGeomGetData(_o2));

            // Figure out which one is a ray; note that this assumes
            // that the ODE dRayClass is used *soley* by the RayCollision.
            if (dGeomGetClass(_o1) == dRayClass)
            {
                hitCollision = collision2;
                dGeomRaySetParams(_o1, 0, 0);
                dGeomRaySetClosestHit(_o1, 1);
            }
            else if (dGeomGetClass(_o2) == dRayClass)
            {
                hitCollision = collision1;
                dGeomRaySetParams(_o2, 0, 0);
                dGeomRaySetClosestHit(_o2, 1);
            }

            if (hitCollision)
            {
                // Check for ray/collision intersections
                int n = dCollide(_o1, _o2, 1, &contact, sizeof(contact));

                if (n > 0)
                {
                    if (contact.depth < inter->depth)
                    {
                    inter->depth = contact.depth;
                    inter->name = hitCollision->GetScopedName();
                    }
                }
            }
        }
    }

    dGeomID geomId;
    ODEPhysicsPtr physicsEngine;

    ignition::math::Vector3d dir;

    class Intersection
    {
        /// \brief Depth of the ray intersection.
        public: double depth;

        /// \brief Name of the collision object that was hit.
        public: std::string name;
    };
};
}
}
