
#include "PSMoveUtils.h"


Tracker::Tracker() : m_moves(NULL), m_count(psmove_count_connected()), mocapRecorder("psmovedata") {
    // PSMove *move;
    // move = psmove_connect();
    // if (move == NULL) {
    //     printf("Could not connect to default Move controller.\n"
    //            "Please connect one via USB or Bluetooth.\n");
    //     exit(1);
    // }
    // char *serial = psmove_get_serial(move);
    // auto ctype = psmove_connection_type(move);
    // switch (ctype) {
    //     case Conn_USB:
    //         printf("Connected via USB.\n");
    //         break;
    //     case Conn_Bluetooth:
    //         printf("Connected via Bluetooth.\n");
    //         break;
    //     case Conn_Unknown:
    //         printf("Unknown connection type.\n");
    //         break;
    // }
    std::cout << "PSMOVE : Initialising Tracker..." << std::endl;
    m_tracker = psmove_tracker_new();
    std::cout << "PSMOVE : Tracker Initialised" << std::endl;

    std::cout << "PSMOVE : Initialising Fusion..." << std::endl;
    m_fusion = psmove_fusion_new(m_tracker, 1., 1000.);
    std::cout << "PSMOVE : Fusion Initialised" << std::endl;

    psmove_tracker_set_mirror(m_tracker, PSMove_False);
    psmove_tracker_set_exposure(m_tracker, Exposure_HIGH);

    m_moves = (PSMove**) calloc(m_count, sizeof(PSMove*));
    m_items = (int*) calloc(m_count, sizeof(int));
    for (int i = 0; i < m_count; i++) {
        std::cout << "PSMOVE : Initialising Controller " << i << "..." << std::endl;
        m_moves[i] = psmove_connect_by_id(i);
        m_items[i] = WIRE_CUBE;

        psmove_enable_orientation(m_moves[i], PSMove_True);
        assert(psmove_has_orientation(m_moves[i]));


        std::cout << "PSMOVE : Calibrating Controller " << i << "..." << std::endl;
        while (psmove_tracker_enable(m_tracker, m_moves[i]) != Tracker_CALIBRATED)
            ;
        std::cout << "PSMOVE : Controller Calibrated " << i << "..." << std::endl;
    }
    // psmove_tracker_set_dimming(m_tracker,1);
    std::cout << "PSMOVE INITIALISED" << std::endl;
}

Tracker::~Tracker() {
    psmove_fusion_free(m_fusion);
    psmove_tracker_free(m_tracker);
    for (int i = 0; i < m_count; i++) {
        psmove_disconnect(m_moves[i]);
    }
    free(m_items);
    free(m_moves);
}

void Tracker::setProjection(glm::mat4 proj) {
    float* proj_ptr = psmove_fusion_get_projection_matrix(m_fusion);
    for (int i = 0; i < 16; i++) {
        std::cout << proj_ptr[i] << std::endl;
    }
    memcpy(proj_ptr, glm::value_ptr(proj), sizeof(float) * 16);
    for (int i = 0; i < 16; i++) {
        std::cout << proj_ptr[i] << std::endl;
    }
}


void Tracker::update() {
    for (int i = 0; i < m_count; i++) {
        while (psmove_poll(m_moves[i]))
            ;

        float x, y, z;
        psmove_fusion_get_position(m_fusion, m_moves[i], &x, &y, &z);

        int buttons = psmove_get_buttons(m_moves[i]);
        if (buttons & Btn_MOVE) {
            psmove_reset_orientation(m_moves[i]);
        }
        else if (buttons & Btn_PS) {
            exit(0);
        }
        else if (buttons & Btn_SELECT) {
            m_rotation += 2.;
        }
        else if (buttons & Btn_CROSS) {
            m_trace.push_back(Point3D(x, y, z));
        }

        if (buttons & Btn_START) {
            if (m_has_last_offset) {
                m_offset = Point3D(m_offset.x + x - m_last_offset.x,
                                   m_offset.y + y - m_last_offset.y,
                                   m_offset.z + z - m_last_offset.z);
            }
            else {
                m_has_last_offset = true;
            }
            m_last_offset = Point3D(x, y, z);
        }
        else {
            m_has_last_offset = false;
        }

        unsigned int pressed, released;
        psmove_get_button_events(m_moves[i], &pressed, &released);
        if (pressed & Btn_SQUARE) {
            m_items[i] -= 1;
            if (m_items[i] < 0) m_items[i] = ITEM_MAX - 1;
        }
        else if (pressed & Btn_TRIANGLE) {
            m_items[i] += 1;
            if (m_items[i] == ITEM_MAX) m_items[i] = 0;
        }
        else if (pressed & Btn_CIRCLE) {
            m_trace.clear();
            m_rotation = 0.;
            m_offset   = Point3D(0., 0., 0.);
        }
    }

    psmove_tracker_update_image(m_tracker);
    psmove_tracker_update(m_tracker, NULL);
}

void Tracker::init() {
    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &m_texture);
    glBindTexture(GL_TEXTURE_2D, m_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
}

void Tracker::render() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    PSMoveTrackerRGBImage image = psmove_tracker_get_image(m_tracker);

    glEnable(GL_TEXTURE_2D);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.width, image.height, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    /* Draw the camera image, filling the screen */
    glColor3f(1., 1., 1.);
    glBegin(GL_QUADS);
    // BL
    glTexCoord2f(0., 1.);
    glVertex2f(-1., -1.);
    // BR
    glTexCoord2f(1., 1.);
    glVertex2f(1., -1.);
    // TR
    glTexCoord2f(1., 0.);
    glVertex2f(1., 1.);
    // TL
    glTexCoord2f(0., 0.);
    glVertex2f(-1., 1.);
    glEnd();

    glDisable(GL_TEXTURE_2D);

    /* Clear the depth buffer to allow overdraw */
    glClear(GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(psmove_fusion_get_projection_matrix(m_fusion));

    /* Render the trace */
    if (m_trace.size()) {
        Point3D center = m_trace[0];
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(center.x + m_offset.x, center.y + m_offset.y, center.z + m_offset.z);
        glRotatef(m_rotation, 0., 1., 0.);

        std::vector<Point3D>::iterator it;
        glColor3f(1., 0., 0.);
        glEnable(GL_LIGHTING);
        // glBegin(GL_TRIANGLE_STRIP);
        for (it = m_trace.begin(); it != m_trace.end(); ++it) {
            Point3D point = *it;
            Point3D moved(point.x - center.x, point.y - center.y, point.z - center.z);
            // glVertex3f(moved.x, moved.y, moved.z);
            glPushMatrix();
            glTranslatef(moved.x, moved.y, moved.z);
            glutSolidCube(.5);
            glPopMatrix();
        }
        // glEnd();
        glDisable(GL_LIGHTING);
    }

    for (int i = 0; i < m_count; i++) {
        glMatrixMode(GL_MODELVIEW);
        GLfloat* modelview = psmove_fusion_get_modelview_matrix(m_fusion, m_moves[i]);
        glLoadMatrixf(modelview);

        if (m_items[i] == WIRE_CUBE) {
            glColor3f(1., 0., 0.);
            glutWireCube(1.);
            glColor3f(0., 1., 0.);

            glPushMatrix();
            glScalef(1., 1., 4.5);
            glTranslatef(0., 0., -.5);
            glutWireCube(1.);
            glPopMatrix();

            glColor3f(0., 0., 1.);
            glutWireCube(3.);
        }
        else if (m_items[i] == SOLID_CUBE) {
            glEnable(GL_LIGHTING);
            glutSolidCube(2.);
            glDisable(GL_LIGHTING);
        }
        else if (m_items[i] == SOLID_TEAPOT) {
            glEnable(GL_LIGHTING);
            glPushMatrix();
            glRotatef(90., 1., 0., 0.);
            glutSolidTeapot(1.);
            glPopMatrix();
            glDisable(GL_LIGHTING);
        }
    }
}

void Tracker::saveFrame(CvVideoWriter* writer) {
    void* frame = psmove_tracker_get_frame(m_tracker);
    if (frame) {
        cvWriteFrame(writer, (const IplImage*) frame);
    }
    else {
        std::cout << "Frame failed to save" << std::endl;
    }
}
void Tracker::addMeasurementsToStream(autocal::SensorPlant& plant, std::string stream_name, autocal::TimeStamp t) {
    for (int i = 0; i < m_count; i++) {
        GLfloat* m = psmove_fusion_get_modelview_matrix(m_fusion, m_moves[i]);
        Transform3D pose;
        // units = decimeters
        // HACK O.5 FOR HALF DISTANCE
        pose << m[0] << m[4] << m[8] << unit_factor * m[12] * 0.5 << arma::endr << m[1] << m[5] << m[9]
             << unit_factor * m[13] * 0.5 << arma::endr << m[2] << m[6] << m[10] << unit_factor * m[14] * 0.5
             << arma::endr << m[3] << m[7] << m[11] << m[15] << arma::endr;

        plant.mocapRecording.addMeasurement(stream_name, t, i, pose);
    }
}

void Tracker::savePoses() {
    std::vector<Transform3D> poses;
    for (int i = 0; i < m_count; i++) {
        GLfloat* m = psmove_fusion_get_modelview_matrix(m_fusion, m_moves[i]);
        Transform3D pose;
        // units = decimeters
        pose << m[0] << m[4] << m[8] << unit_factor * m[12] << arma::endr << m[1] << m[5] << m[9] << unit_factor * m[13]
             << arma::endr << m[2] << m[6] << m[10] << unit_factor * m[14] << arma::endr << m[3] << m[7] << m[11]
             << m[15] << arma::endr;

        // TODO:
        poses.push_back(pose);
        // std::cout << pose << std::endl;
    }
    mocapRecorder.saveFrame(poses);
}
