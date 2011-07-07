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
    isLost = true;
    timeLastLostUpdate = 0;
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
        isLost = true;
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
    if (timeSeen > 40)
        isLost = false;
    else if (timestamp - timeLastLostUpdate < 3000)
        isLost = false;
    else if (timeSinceLastSeen > 5000)
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
void MobileObject::updateIsLost(bool islost, double timestamp)
{
    isLost = islost;
    if (not isLost)
        timeLastLostUpdate = timestamp;
}

/*! @brief Updates the time last seen to the given value 
    DO NOT USE. This funcion is just for jason.
 */
void MobileObject::updateTimeLastSeen(float time)
{
    timeLastSeen = time;
}

std::ostream& operator<< (std::ostream& output, const MobileObject& p_mob)
{
    output << *static_cast<const Object*>(&p_mob);

    output.write(reinterpret_cast<const char*>(&p_mob.estimatedFieldLocationError.x), sizeof(p_mob.estimatedFieldLocationError.x));
    output.write(reinterpret_cast<const char*>(&p_mob.estimatedFieldLocationError.y), sizeof(p_mob.estimatedFieldLocationError.y));

    output.write(reinterpret_cast<const char*>(&p_mob.estimatedVelocity.x), sizeof(p_mob.estimatedVelocity.x));
    output.write(reinterpret_cast<const char*>(&p_mob.estimatedVelocity.x), sizeof(p_mob.estimatedVelocity.x));

    output.write(reinterpret_cast<const char*>(&p_mob.estimatedVelocityError.x), sizeof(p_mob.estimatedVelocityError.x));
    output.write(reinterpret_cast<const char*>(&p_mob.estimatedVelocityError.y), sizeof(p_mob.estimatedVelocityError.y));

    output.write(reinterpret_cast<const char*>(&p_mob.sharedCovariance[0][0]), sizeof(p_mob.sharedCovariance[0][0]));
    output.write(reinterpret_cast<const char*>(&p_mob.sharedCovariance[0][1]), sizeof(p_mob.sharedCovariance[0][1]));
    output.write(reinterpret_cast<const char*>(&p_mob.sharedCovariance[1][0]), sizeof(p_mob.sharedCovariance[1][0]));
    output.write(reinterpret_cast<const char*>(&p_mob.sharedCovariance[1][1]), sizeof(p_mob.sharedCovariance[1][1]));

    output.write(reinterpret_cast<const char*>(&p_mob.isLost), sizeof(p_mob.isLost));

    return output;
}

std::istream& operator>> (std::istream& input, MobileObject& p_mob)
{
    input >> *static_cast<Object*>(&p_mob);

    p_mob.sharedCovariance = Matrix(2,2,false);

    input.read(reinterpret_cast<char*>(&p_mob.estimatedFieldLocationError.x), sizeof(p_mob.estimatedFieldLocationError.x));
    input.read(reinterpret_cast<char*>(&p_mob.estimatedFieldLocationError.y), sizeof(p_mob.estimatedFieldLocationError.y));

    input.read(reinterpret_cast<char*>(&p_mob.estimatedVelocity.x), sizeof(p_mob.estimatedVelocity.x));
    input.read(reinterpret_cast<char*>(&p_mob.estimatedVelocity.x), sizeof(p_mob.estimatedVelocity.x));

    input.read(reinterpret_cast<char*>(&p_mob.estimatedVelocityError.x), sizeof(p_mob.estimatedVelocityError.x));
    input.read(reinterpret_cast<char*>(&p_mob.estimatedVelocityError.y), sizeof(p_mob.estimatedVelocityError.y));

    input.read(reinterpret_cast<char*>(&p_mob.sharedCovariance[0][0]), sizeof(p_mob.sharedCovariance[0][0]));
    input.read(reinterpret_cast<char*>(&p_mob.sharedCovariance[0][1]), sizeof(p_mob.sharedCovariance[0][1]));
    input.read(reinterpret_cast<char*>(&p_mob.sharedCovariance[1][0]), sizeof(p_mob.sharedCovariance[1][0]));
    input.read(reinterpret_cast<char*>(&p_mob.sharedCovariance[1][1]), sizeof(p_mob.sharedCovariance[1][1]));

    input.read(reinterpret_cast<char*>(&p_mob.isLost), sizeof(p_mob.isLost));

    return input;
}
