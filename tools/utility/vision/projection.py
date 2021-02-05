import numpy as np

# A Pythonic Class of Lens Transforms

# TODO these really should come from the protocol buffer definitions
RECTILINEAR_PROJECTION = 1
EQUIDISTANT_PROJECTION = 2
EQUISOLID_PROJECTION = 3


def _inverseCoefficients(k):
    return [
        -k[0],
        3.0 * (k[0] * k[0]) - k[1],
        -12.0 * (k[0] * k[0]) * k[0] + 8.0 * k[0] * k[1],
        55.0 * (k[0] * k[0]) * (k[0] * k[0]) - 55.0 * (k[0] * k[0]) * k[1] + 5.0 * (k[1] * k[1]),
    ]


#
# Takes an undistorted radial distance from the optical axis and computes and applies an inverse distortion
#
#  @param r the radius to undistort
#  @param k the undistortion coefficients
#
#  @return an distortion radius
#
def _distort(r, k):
    ik = _inverseCoefficients(k)
    return r * (
        1.0
        + ik[0] * (r * r)
        + ik[1] * ((r * r) * (r * r))
        + ik[2] * ((r * r) * (r * r)) * (r * r)
        + ik[3] * ((r * r) * (r * r)) * ((r * r) * (r * r))
    )


#
# Takes a distorted radial distance from the optical axis and applies the polynomial distortion coefficients to undistort it
#
#  @param r the radius to undistort
#  @param k the undistortion coefficients
#
#  @return an undistorted radius
#
def _undistort(r, k):
    # These parenthesis are important as they allow the compiler to optimise further
    return r * (1.0 + k[0] * (r * r) + k[1] * (r * r) * (r * r))


def _equidistantR(theta, f):
    return f * theta


def _equidistantTheta(r, f):
    return r / f


def _rectilinearR(theta, f):
    return f * np.tan(np.clip(theta, 0, np.pi * 0.5), dtype="f8")


def _rectilinearTheta(r, f):
    return np.arctan2(r, f, dtype="f8")


def _equisolidR(theta, f):
    return 2.0 * f * np.sin(theta * 0.5, dtype="f8")


def _equisolidTheta(r, f):
    return 2.0 * np.arcsin(r / (2.0 * f), dtype="f8")


def project(ray, projection, f, centre, k, dimensions):

    # Perform the projection math from Ray to screen:
    # https://www.plunk.org/~hatch/rightway.php (angle between unit vectors)
    # change from: theta = np.arccos(ray[0])
    theta = 2.0 * np.arctan2(np.linalg.norm(ray - [1, 0, 0]), np.linalg.norm(ray + [1, 0, 0]))
    rsinTheta = 1.0 / np.sqrt(1.0 - np.square(ray[0]))
    if projection == RECTILINEAR_PROJECTION:
        rU = _rectilinearR(theta, f)
    elif projection == EQUISOLID_PROJECTION:
        rU = _equisolidR(theta, f)
    elif projection == EQUIDISTANT_PROJECTION:
        rU = _equidistantR(theta, f)
    else:
        raise RuntimeError("Unknown projection type: {}".format(projection))

    rD = _distort(rU, k)
    screen = np.array([rD * ray[1] * rsinTheta, rD * ray[2] * rsinTheta])

    return (np.array(dimensions) * 0.5) - screen - centre


def unproject(px, projection, f, centre, k, dimensions):

    # Unproject the pixel coordinates back into unit vector rays
    # Perform the inverse projection math:

    screen = (np.array(dimensions) * 0.5) - px - centre
    rD = np.linalg.norm(screen)

    rU = _undistort(rD, k)
    if projection == RECTILINEAR_PROJECTION:
        theta = _rectilinearTheta(rU, f)
    elif projection == EQUISOLID_PROJECTION:
        theta = _equisolidTheta(rU, f)
    elif projection == EQUIDISTANT_PROJECTION:
        theta = _equidistantTheta(rU, f)
    else:
        raise RuntimeError("Unknown projection type: {}".format(projection))

    t = np.sin(theta) * screen / rD
    return np.array([np.cos(theta), t[0], t[1]])
