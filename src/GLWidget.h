#ifndef GLWidget_h
#define GLWidget_h

#include <QtOpenGL>
#include "Pose2.h"

class GLWidget : public QGLWidget
{
    Q_OBJECT

public:

    GLWidget(QWidget* parent = 0);
    GLWidget(QGLContext* context, QWidget* parent = 0, const QGLWidget* shareWidget = 0, Qt::WindowFlags f = 0);
    GLWidget(const QGLFormat& format, QWidget* parent = 0, const QGLWidget* shareWidget = 0, Qt::WindowFlags f = 0);

    QSize sizeHint() const { return QSize(500, 500); }

    const Pose2_disc& disc_min() const { return min_; }
    const Pose2_disc& disc_max() const { return max_; }

    bool discrete_mode() const { return disc_mode_; }

    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);

public slots:

    void toggle_disc_mode();
    void set_num_angles(int);
    void set_disc_start_angle(int);
    void set_disc_goal_angle(int);
    void set_disc_goal_x(int);
    void set_disc_goal_y(int);

private:

    bool disc_mode_;

    bool left_button_down_;
    bool right_button_down_;

    bool draw_arrows_;

    QPointF start_tail_;
    QPointF goal_tail_;

    QPointF start_head_;
    QPointF goal_head_;

    Pose2_disc min_;
    Pose2_disc max_;

    int num_angles_;

    QPointF viewport_to_world(const QPointF& viewport_coord) const;

    void draw_grid();
    void draw_arrow(double x, double y, double yaw, double r, double g, double b);
    void draw_line(const std::vector<Pose2_cont>& motion);

    void draw_discrete_neighbors();
    void draw_widest_arcs();

    double realize_angle(int disc_angle, int num_angles);
};

#endif
