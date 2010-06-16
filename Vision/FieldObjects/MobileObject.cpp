#include "MobileObject.h"

MobileObject::MobileObject(int initID, const std::string& initName):
        Object(initID, initName)
{
    estimatedFieldLocationError[0] = 0;
    estimatedFieldLocationError[1] = 0;
    estimatedVelocity[0] = 0;
    estimatedVelocity[1] = 0;
    estimatedVelocityError[0] = 0;
    estimatedVelocityError[1] = 0;
    
    sharedCovariance = Matrix(2,2);
    sharedCovariance[0][0] = 600;
    sharedCovariance[0][1] = 0;
    sharedCovariance[1][1] = 600;
    isLost = false;
}

MobileObject::MobileObject(const Vector2<float>& newEstimatedLocation, int initID, const std::string& initName):
                Object(initID, initName),
                estimatedFieldLocation(newEstimatedLocation)
{
        estimatedFieldLocationError[0] = 0;
        estimatedFieldLocationError[1] = 0;
        estimatedVelocity[0] = 0;
        estimatedVelocity[1] = 0;
        estimatedVelocityError[0] = 0;
        estimatedVelocityError[1] = 0;	
    
        sharedCovariance = Matrix(2,2);
        sharedCovariance[0][0] = 600;
        sharedCovariance[0][1] = 0;
        sharedCovariance[1][1] = 600;
        isLost = false;
}

MobileObject::MobileObject(const MobileObject& srcObj):
        Object(srcObj.getID(), srcObj.getName()),
        estimatedFieldLocation(srcObj.getEstimatedFieldLocation()),
        estimatedFieldLocationError(srcObj.getEstimatedFieldLocationError()),
        estimatedVelocity(srcObj.getEstimatedVelocity()),
        estimatedVelocityError(srcObj.getEstimatedVelocityError()),
        sharedCovariance(srcObj.sharedCovariance),
        isLost(srcObj.isLost)
{
}

MobileObject::~MobileObject()
{
}

/*! @brief Postprocess the field object after updating visual information
    @param timestamp the current image timestamp

    This function sets everything Object::postProcess does, as well as the isLost flag.
 */
void MobileObject::postProcess(const float timestamp)
{
    Object::postProcess(timestamp);
    if (timeSinceLastSeen < 250)
        isLost = false;
    else
        isLost = true;
}

void MobileObject::updateAbsoluteLocation(const Vector2<float>& newAbsoluteLocation)
{
        estimatedFieldLocation = newAbsoluteLocation;
}
void MobileObject::updateAbsoluteLocationError(const Vector2<float>& newAbsoluteLocationError)
{
        estimatedFieldLocationError = newAbsoluteLocationError;
}
void MobileObject::updateObjectLocation(float x, float y, float sdx, float sdy)
{
        estimatedFieldLocation[0] = x;
        estimatedFieldLocation[1] = y;
        estimatedFieldLocationError[0] = sdx;
        estimatedFieldLocationError[1] = sdy;
}

void MobileObject::updateVelocity(const Vector2<float>& newVelocity)
{
        estimatedVelocity = newVelocity;
}
void MobileObject::updateVelocityError(const Vector2<float>& newVelocityError)
{
        estimatedVelocityError = newVelocityError;
	
}
void MobileObject::updateObjectVelocities(float velX, float velY, float sdVelX, float sdVelY)
{
        estimatedVelocity[0] = velX;
        estimatedVelocity[1] = velY;
        estimatedVelocityError[0] = sdVelX;
        estimatedVelocityError[1] = sdVelY;
}

/*! @brief Updates the shared covariance matrix for this mobile object */
void MobileObject::updateSharedCovariance(const Matrix& sharedSR)
{
    sharedCovariance = sharedSR;
}

/*! @brief Updates whether this mobile object is lost */
void MobileObject::updateIsLost(bool islost)
{
    isLost = islost;
}

std::ostream& operator<< (std::ostream& output, const MobileObject& p_mob)
{
    output << *static_cast<const Object*>(&p_mob);
    output << p_mob.estimatedFieldLocationError.x << ' ' << p_mob.estimatedFieldLocationError.y << ' ';
    output << p_mob.estimatedVelocity.x << ' ' << p_mob.estimatedVelocity.y << ' ';
    output << p_mob.estimatedVelocityError.x << ' ' << p_mob.estimatedVelocityError.y << ' ';
    output << p_mob.sharedCovariance[0][0] << ' ' << p_mob.sharedCovariance[0][1] << ' ' << p_mob.sharedCovariance[1][0] << ' ' << p_mob.sharedCovariance[1][1] << ' ';
    output << p_mob.isLost << ' ';
}

std::istream& operator>> (std::istream& input, MobileObject& p_mob)
{
    input >> *static_cast<Object*>(&p_mob);
    input >> p_mob.estimatedFieldLocationError.x >> p_mob.estimatedFieldLocationError.y;
    input >> p_mob.estimatedVelocity.x >> p_mob.estimatedVelocity.y;
    input >> p_mob.estimatedVelocityError.x >> p_mob.estimatedVelocityError.y;
    p_mob.sharedCovariance = Matrix(2,2,false);
    input >> p_mob.sharedCovariance[0][0] >> p_mob.sharedCovariance[0][1] >> p_mob.sharedCovariance[1][0] >> p_mob.sharedCovariance[1][1];
    input >> p_mob;
}
