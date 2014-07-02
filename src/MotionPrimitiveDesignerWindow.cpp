#include "MotionPrimitiveDesignerWindow.h"
#include <cstdio>
#include "GLWidget.h"
#include "DiscreteAnglesSpinBox.h"

MotionPrimitiveDesignerWindow::MotionPrimitiveDesignerWindow(QWidget* parent, Qt::WindowFlags flags) :
    QMainWindow(parent, flags)
{
    QGLFormat format;
    format.setSampleBuffers(true);
    render_widget_ = new GLWidget(format, this);

    control_panel_dock_widget_ = new QDockWidget(this);
    control_panel_dock_widget_->setAllowedAreas(Qt::LeftDockWidgetArea);
    control_panel_dock_widget_->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
    addDockWidget(Qt::LeftDockWidgetArea, control_panel_dock_widget_);

    QWidget* control_panel_widget = new QWidget;
    control_panel_dock_widget_->setWidget(control_panel_widget);

    QVBoxLayout* control_panel_layout = new QVBoxLayout;

    discrete_mode_toggle_button_ = new QPushButton(tr("Toggle Discrete Mode"));
    num_disc_angles_spinbox_ = new DiscreteAnglesSpinBox;
    start_disc_angle_spinbox_ = new QSpinBox;
    goal_disc_angle_spinbox_ = new QSpinBox;
    goal_disc_x_spinbox_ = new QSpinBox;
    goal_disc_y_spinbox_ = new QSpinBox;

    control_panel_layout->addWidget(discrete_mode_toggle_button_, Qt::AlignTop);
    control_panel_layout->addWidget(num_disc_angles_spinbox_, Qt::AlignTop);
    control_panel_layout->addWidget(start_disc_angle_spinbox_ /*, Qt::AlignTop*/);
    control_panel_layout->addWidget(goal_disc_angle_spinbox_ /*, Qt::AlignTop*/);
    control_panel_layout->addWidget(goal_disc_x_spinbox_ /*, Qt::AlignTop*/);
    control_panel_layout->addWidget(goal_disc_y_spinbox_ /*, Qt::AlignTop*/);
    control_panel_layout->addStretch();

    control_panel_widget->setLayout(control_panel_layout);

    setCentralWidget(render_widget_);

    connect(discrete_mode_toggle_button_, SIGNAL(clicked()), this, SLOT(toggle_selection_mode()));
    connect(num_disc_angles_spinbox_, SIGNAL(valueChanged(int)), this, SLOT(update_num_angles(int)));
    connect(start_disc_angle_spinbox_, SIGNAL(valueChanged(int)), render_widget_, SLOT(set_disc_start_angle(int)));
    connect(goal_disc_angle_spinbox_, SIGNAL(valueChanged(int)), render_widget_, SLOT(set_disc_goal_angle(int)));
    connect(goal_disc_x_spinbox_, SIGNAL(valueChanged(int)), render_widget_, SLOT(set_disc_goal_x(int)));
    connect(goal_disc_y_spinbox_, SIGNAL(valueChanged(int)), render_widget_, SLOT(set_disc_goal_y(int)));

    num_disc_angles_spinbox_->setMinimum(1);
    num_disc_angles_spinbox_->setMaximum(256);

    goal_disc_x_spinbox_->setMinimum(render_widget_->disc_min().x);
    goal_disc_x_spinbox_->setMaximum(render_widget_->disc_max().x);
    goal_disc_y_spinbox_->setMinimum(render_widget_->disc_min().y);
    goal_disc_y_spinbox_->setMaximum(render_widget_->disc_max().y);

    last_spinbox_value_ = 16;
    num_disc_angles_spinbox_->setValue(last_spinbox_value_);

    render_widget_->set_num_angles(16);
}

void MotionPrimitiveDesignerWindow::update_num_angles(int i)
{
    start_disc_angle_spinbox_->setMaximum(i - 1);
    goal_disc_angle_spinbox_->setMaximum(i - 1);
    render_widget_->set_num_angles(i);
}

bool MotionPrimitiveDesignerWindow::is_pow2(unsigned i)
{
    return i != 0 && !(i & (i - 1));
}

void MotionPrimitiveDesignerWindow::toggle_selection_mode()
{
    render_widget_->toggle_disc_mode();
    discrete_mode_toggle_button_->setText(QString("Toggle %1 Mode").arg(render_widget_->discrete_mode() ? "Continuous" : "Discrete"));
}