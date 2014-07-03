#include <cmath>
#include <cstdio>
#include <GL/gl.h>
#include <GL/glu.h>
#include "GLWidget.h"
#include "unicycle_motions.h"

static int discretize(double d, double res)
{
    return (int)(d / res);
}

static double continuize(int i, double res)
{
    return (double)(i * res);
}

static const double INITIAL_START_HEADING = 11.25 * M_PI / 180.0;
static const double INITIAL_GOAL_HEADING = 0 * M_PI / 180.0;

static const int MOTION_DX = 10;
static const int MOTION_DY = 1;
static const int APPROX_DIST = sqrt((double)(MOTION_DX * MOTION_DX + MOTION_DY * MOTION_DY));
static const double INITIAL_GOAL_X = APPROX_DIST * cos(INITIAL_START_HEADING);
static const double INITIAL_GOAL_Y = APPROX_DIST * sin(INITIAL_START_HEADING);

GLWidget::GLWidget(QWidget* parent) :
    QGLWidget(parent),
    disc_mode_(true),
    min_(-15, -15, 0),
    max_(15, 15, 360),
    left_button_down_(false),
    right_button_down_(false),
    start_tail_(0.0, 0.0),
    goal_tail_(INITIAL_GOAL_X, INITIAL_GOAL_Y),
    start_head_(start_tail_.x() + cos(INITIAL_START_HEADING), start_tail_.y() + sin(INITIAL_START_HEADING)),
    goal_head_(goal_tail_.x() + cos(INITIAL_GOAL_HEADING), goal_tail_.y() + sin(INITIAL_GOAL_HEADING)),
    draw_arrows_(true),
    num_angles_(16)
{
}

GLWidget::GLWidget(QGLContext* context, QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f) :
    QGLWidget(context, parent, shareWidget, f),
    disc_mode_(true),
    min_(-15, -15, 0),
    max_(15, 15, 360),
    left_button_down_(false),
    right_button_down_(false),
    start_tail_(0.0, 0.0),
    goal_tail_(INITIAL_GOAL_X, INITIAL_GOAL_Y),
    start_head_(start_tail_.x() + cos(INITIAL_START_HEADING), start_tail_.y() + sin(INITIAL_START_HEADING)),
    goal_head_(goal_tail_.x() + cos(INITIAL_GOAL_HEADING), goal_tail_.y() + sin(INITIAL_GOAL_HEADING)),
    draw_arrows_(true),
    num_angles_(16)
{

}

GLWidget::GLWidget(const QGLFormat& format, QWidget* parent, const QGLWidget* shareWidget, Qt::WindowFlags f) :
    QGLWidget(format, parent, shareWidget, f),
    disc_mode_(true),
    min_(-15, -15, 0),
    max_(15, 15, 360),
    left_button_down_(false),
    right_button_down_(false),
    start_tail_(0.0, 0.0),
    goal_tail_(INITIAL_GOAL_X, INITIAL_GOAL_Y),
    start_head_(start_tail_.x() + cos(INITIAL_START_HEADING), start_tail_.y() + sin(INITIAL_START_HEADING)),
    goal_head_(goal_tail_.x() + cos(INITIAL_GOAL_HEADING), goal_tail_.y() + sin(INITIAL_GOAL_HEADING)),
    draw_arrows_(true),
    num_angles_(16)
{

}

void GLWidget::initializeGL()
{
    glClearColor(1.0f, 0.98f, 0.98f, 1.0f);
    // glClearColor(1.0f, 1.0f, 0.941f, 1.0f);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    draw_grid();

    const int num_angles = 16;
    auto interp = [](double from, double to, double a) { return (1.0 - a) * from + a * to; };

    if (!disc_mode_)
    {
        // draw the set start/goal guidelines
        glColor3f(0.0f, 0.0f, 1.0f);
        if (left_button_down_) {
            glBegin(GL_LINES);
            glVertex2d(start_tail_.x(), start_tail_.y());
            glVertex2d(start_head_.x(), start_head_.y());
            glEnd();
        }
        if (right_button_down_) {
            glBegin(GL_LINES);
            glVertex2d(goal_tail_.x(), goal_tail_.y());
            glVertex2d(goal_head_.x(), goal_head_.y());
            glEnd();
        }
    }

    double start_angle = atan2(start_head_.y() - start_tail_.y(), start_head_.x() - start_tail_.x());
    Pose2_cont start = { start_tail_.x(), start_tail_.y(), start_angle };

    std::vector<Pose2_cont> poses;

    double goal_angle = atan2(goal_head_.y() - goal_tail_.y(), goal_head_.x() - goal_tail_.x());
    poses.push_back({ goal_tail_.x(), goal_tail_.y(), goal_angle });
    for (const Pose2_cont goal : poses)
    {
        std::vector<Pose2_cont> motion = generate_unicycle_motion(start, goal);
        draw_line(motion);
    }

    // draw the start
    draw_arrow(start.x, start.y, start.yaw, 0.0, 1.0, 0.0);

    // draw the goal
    draw_arrow(poses.front().x, poses.front().y, poses.front().yaw, 1.0, 0.0, 0.0);

    glFlush();
    swapBuffers();
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    if (disc_mode_) {
        return;
    }

    if (event->button() == Qt::LeftButton) {
        start_tail_ = viewport_to_world(event->posF());
        printf("Selected Start Tail: (%0.3f, %0.3f)\n", start_tail_.x(), start_tail_.y());
        left_button_down_ = true;
        update();
    }
    else if (event->button() == Qt::RightButton) {
        goal_tail_ = viewport_to_world(event->posF());
        printf("Selected Goal Tail: (%0.3f, %0.3f)\n", goal_tail_.x(), goal_tail_.y());
        right_button_down_ = true;
        update();
    }
    else {
        draw_arrows_ = !draw_arrows_;
    }
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (disc_mode_) {
        return;
    }

    if (left_button_down_) {
        start_head_ = viewport_to_world(event->posF());
    }
    if (right_button_down_) {
        goal_head_ = viewport_to_world(event->posF());
    }
    update();
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (disc_mode_) {
        return;
    }

    if (event->button() == Qt::LeftButton) {
        left_button_down_ = false;
    }
    else if (event->button() == Qt::RightButton) {
        right_button_down_ = false;
    }
    update();
}

void GLWidget::toggle_disc_mode()
{
    disc_mode_ = !(disc_mode_);

    printf("Toggle Discrete Mode: %s!\n", (disc_mode_ ? "On" : "Off"));

    if (disc_mode_) {
        // snap to nearest discrete goal state
        double goaldx = goal_head_.x() - goal_tail_.x();
        double goaldy = goal_head_.y() - goal_tail_.y();
        double goalangle = atan2(goaldy, goaldx);

        double goal_x = std::round(goal_tail_.x());
        double goal_y = std::round(goal_tail_.y());
        double goal_angle = realize_angle(discretize_angle(goalangle, num_angles_), num_angles_);

        goal_tail_ = QPointF(goal_x, goal_y);
        goal_head_ = QPointF(goal_x + cos(goal_angle), goal_y + sin(goal_angle));

        // snap to nearest discrete start state
        double startdx = start_head_.x() - start_tail_.x();
        double startdy = start_head_.y() - start_tail_.y();
        double startangle = atan2(startdy, startdx);

        double start_x = std::round(start_tail_.x());
        double start_y = std::round(start_tail_.y());
        double start_angle = realize_angle(discretize_angle(startangle, num_angles_), num_angles_);

        start_tail_ = QPointF(start_x, start_y);
        start_head_ = QPointF(start_x + cos(start_angle), start_y + sin(start_angle));

        // TODO: update the gui to reflect the new discrete values
    }

    left_button_down_ = false;
    right_button_down_ = false;

    update();
}

void GLWidget::set_num_angles(int num_angles)
{
    printf("Set Num Angles to %d!\n", num_angles);
    num_angles_ = num_angles;
    update();
}

void GLWidget::set_disc_start_angle(int angle)
{
    printf("Set Discrete Start Angle to %d!\n", angle);
    start_head_ = QPointF(start_tail_.x() + cos(realize_angle(angle, num_angles_)),
                          start_tail_.y() + sin(realize_angle(angle, num_angles_)));
    update();
}

void GLWidget::set_disc_goal_angle(int angle)
{
    printf("Set Discrete Goal Angle to %d!\n", angle);
    goal_head_ = QPointF(goal_tail_.x() + cos(realize_angle(angle, num_angles_)),
                         goal_tail_.y() + sin(realize_angle(angle, num_angles_)));
    update();
}

void GLWidget::set_disc_goal_x(int disc_x)
{
    double dx = goal_head_.x() - goal_tail_.x();
    double dy = goal_head_.y() - goal_tail_.y();
    double angle = atan2(dy, dx);

    goal_tail_.setX((double)disc_x);
    goal_head_ = QPointF(goal_tail_.x() + cos(angle), goal_tail_.y() + sin(angle));

    update();
}

void GLWidget::set_disc_goal_y(int disc_y)
{
    double dx = goal_head_.x() - goal_tail_.x();
    double dy = goal_head_.y() - goal_tail_.y();
    double angle = atan2(dy, dx);

    goal_tail_.setY((double)disc_y);
    goal_head_ = QPointF(goal_tail_.x() + cos(angle), goal_tail_.y() + sin(angle));

    update();
}

void GLWidget::resizeGL(int width, int height)
{
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D((GLdouble)min_.x, (GLdouble)max_.x, (GLdouble)min_.y, (GLdouble)max_.y);
    glMatrixMode(GL_MODELVIEW);
}

QPointF GLWidget::viewport_to_world(const QPointF& viewport_coord) const
{
    const double world_width = max_.x - min_.x;
    const double world_height = max_.y - min_.y;
    const double world_x = world_width * viewport_coord.x() / width() + min_.x;
    const double world_y = world_height * (1.0 - viewport_coord.y() / height()) + min_.y;
    return QPointF(world_x, world_y);
}

void GLWidget::draw_grid()
{
    // draw the grid
    glBegin(GL_LINES);
    glColor3f(0.8f, 0.8f, 0.8f);
    for (int x = min_.x; x <= max_.x; ++x) {
        if (x != 0 && (x % 5)) {
            glVertex2d((double)x, min_.y);
            glVertex2d((double)x, max_.y);
        }
    }
    for (int y = min_.y; y <= max_.y; ++y) {
        if (y != 0 && (y % 5)) {
            glVertex2d(min_.x, y);
            glVertex2d(max_.x, y);
        }
    }

    glColor3f(0.5f, 0.5f, 0.5f);
    for (int x = min_.x; x <= max_.x; ++x) {
        if ((x % 5) == 0) {
            glVertex2d((double)x, min_.y);
            glVertex2d((double)x, max_.y);
        }
    }
    for (int y = min_.y; y <= max_.y; ++y) {
        if (y % 5 == 0) {
            glVertex2d(min_.x, y);
            glVertex2d(max_.x, y);
        }
    }

    glColor3f(0.0f, 0.0f, 0.0f);
    glVertex2d(0, min_.y);
    glVertex2d(0, max_.y);
    glVertex2d(min_.x, 0);
    glVertex2d(max_.x, 0);
    glEnd();
}

void GLWidget::draw_arrow(double x, double y, double yaw, double r, double g, double b)
{
    glPushMatrix();
    glColor3f(r, g, b);
    glLoadIdentity();
    glTranslated(x, y, 0);
    glRotated(yaw * 180.0 / M_PI, 0.0, 0.0, 1.0);
    glBegin(GL_TRIANGLES);
    glVertex2d(-0.5, 0.15);
    glVertex2d(-0.5, -0.15);
    glVertex2d(0.666 - 0.5, -0.15);

    glVertex2d(0.666 - 0.5, -0.15);
    glVertex2d(0.666 - 0.5, 0.15);
    glVertex2d(-0.5, 0.15);

    glVertex2d(0.666 - 0.6, 0.3);
    glVertex2d(0.666 - 0.6, -0.3);
    glVertex2d(0.5, 0.0);
    glEnd();
    glPopMatrix();
}

void GLWidget::draw_line(const std::vector<Pose2_cont>& motion)
{
    glColor3f(1.0f, 0.0f, 1.0f);
    glBegin(GL_LINE_STRIP);
    for (const Pose2_cont& pose : motion) {
        glVertex2d(pose.x, pose.y);
    }
    glEnd();
}

void GLWidget::draw_discrete_neighbors()
{
/*
    const int num_angles = 16;
    auto realize_angle = [](int index, int num_angles) { return index * 2.0 * M_PI / num_angles; };
    auto interp = [](double from, double to, double a) { return (1.0 - a) * from + a * to; };

    // draw the set start/goal guidelines
    glColor3f(0.0f, 0.0f, 1.0f);
    if (left_button_down_) {
        glBegin(GL_LINES);
        glVertex2d(start_tail_.x(), start_tail_.y());
        glVertex2d(start_head_.x(), start_head_.y());
        glEnd();
    }
    if (right_button_down_) {
        glBegin(GL_LINES);
        glVertex2d(goal_tail_.x(), goal_tail_.y());
        glVertex2d(goal_head_.x(), goal_head_.y());
        glEnd();
    }

    Pose2_cont start = { start_tail_.x(),
                         start_tail_.y(),
                         atan2(start_head_.y() - start_tail_.y(), start_head_.x() - start_tail_.x()) };
    draw_arrow(start.x, start.y, start.yaw);

    // create goal poses from last mouse event
    std::vector<Pose2_cont> poses;

    double nominal_heading = atan2(goal_head_.y() - goal_tail_.y(), goal_head_.x() - goal_tail_.x());

    poses.push_back({ goal_tail_.x(), goal_tail_.y(), nominal_heading });

    double floor_x = std::floor(goal_tail_.x());
    double floor_y = std::floor(goal_tail_.y());
    double ceil_x = std::ceil(goal_tail_.x());
    double ceil_y = std::ceil(goal_tail_.y());
    poses.push_back({ floor_x, floor_y, nominal_heading });
    poses.push_back({ floor_x, ceil_y, nominal_heading });
    poses.push_back({ ceil_x, floor_y, nominal_heading });
    poses.push_back({ ceil_x, ceil_y, nominal_heading });

    // for (const Pose2_cont goal : poses)
    {
        draw_arrow(poses.front().x, poses.front().y, poses.front().yaw);
    }

    for (const Pose2_cont goal : poses)
    {
        std::vector<Pose2_cont> motion = generate_unicycle_motion(start, goal);
        draw_line(motion);
    }
    */
}

void GLWidget::draw_widest_arcs()
{
/*
    const int num_angles = 16;
    auto realize_angle = [](int index, int num_angles) { return index * 2.0 * M_PI / num_angles; };
    auto interp = [](double from, double to, double a) { return (1.0 - a) * from + a * to; };
    Pose2_cont start(0.0, 1.0, 0.0);
    std::vector<std::vector<Pose2_cont>> curves;
    for (double x = min_.x; x <= max_.x; x += 1.0) {
        for (double y = min_.y; y <= max_.y; y += 1.0)   {
            // find the most significant angle change that still gives us a curve at this point
            // int best_dangle_diff = -1;
            int best_dangle_diff = num_angles;
            std::vector<Pose2_cont> most_outrageous_motion;
            for (int aind = 0; aind < num_angles; ++aind) {
                if (aind == (num_angles >> 1) - 1) {
                    continue;
                }
                int danglediff = std::min(aind, num_angles - aind);
                // if (danglediff > best_dangle_diff) {
                if (danglediff < best_dangle_diff) {
                    Pose2_cont goal = { x, y, realize_angle(aind, num_angles) };
                    std::vector<Pose2_cont> motion = generate_unicycle_motion(start, goal);
                    if (!motion.empty()) {
                        best_dangle_diff = danglediff;
                        most_outrageous_motion = std::move(motion);
                    }
                }
            }

            if (!most_outrageous_motion.empty()) {
                const Pose2_cont& goal = most_outrageous_motion.back();
                if (draw_arrows_) {
                    draw_arrow(goal.x, goal.y, goal.yaw);
                }
                draw_line(most_outrageous_motion);
            }
        }
    }
*/
}

double GLWidget::realize_angle(int index, int num_angles)
{
    return index * (2.0 * M_PI) / num_angles;
}

int GLWidget::discretize_angle(double angle, int num_angles)
{
    double thetaBinSize = 2.0 * M_PI / num_angles;
    return (int)(normalize_angle(angle + thetaBinSize / 2.0) / (2.0 * M_PI) * (num_angles));
}

double GLWidget::normalize_angle(double angle)
{
    // get to the range from -2PI, 2PI
    if (fabs(angle) > 2 * M_PI) {
        angle = angle - ((int)(angle / (2 * M_PI))) * 2 * M_PI;
    }

    // get to the range 0, 2PI
    if (angle < 0) {
        angle += 2 * M_PI;
    }

    return angle;
}
