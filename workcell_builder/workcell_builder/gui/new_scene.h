#ifndef NEWSCENE_H
#define NEWSCENE_H

#include <QDialog>
#include "scene.h"
#include <QListWidgetItem>

namespace Ui {
class NewScene;
}

class NewScene : public QDialog
{
    Q_OBJECT

public:

    bool editing_mode;
    bool success;

    Scene scene;
    void LoadScene(Scene scene);
    explicit NewScene(QWidget *parent = nullptr);
    ~NewScene();

private slots:
    std::vector<std::string> GetLinks(std::string filename);
    int ErrorCheckOrigin(int robot_or_ee);
    void on_add_object_clicked();
    void add_desc_links(QString OutputFolder,int ee_or_robot);
    void on_object_list_itemDoubleClicked(QListWidgetItem *item);
    void on_load_robot_desc_clicked();
    void on_load_ee_desc_clicked();
    void on_add_ext_joint_clicked();
    void on_delete_object_clicked();
    void on_ext_joint_list_itemDoubleClicked(QListWidgetItem *item);
    void on_del_ext_joint_clicked();

    void on_robot_x_textChanged(const QString &arg1);
    void on_robot_y_textChanged(const QString &arg1);
    void on_robot_z_textChanged(const QString &arg1);
    void on_robot_roll_textChanged(const QString &arg1);
    void on_robot_pitch_textChanged(const QString &arg1);
    void on_robot_yaw_textChanged(const QString &arg1);

    void on_ee_x_textChanged(const QString &arg1);
    void on_ee_y_textChanged(const QString &arg1);
    void on_ee_z_textChanged(const QString &arg1);
    void on_ee_roll_textChanged(const QString &arg1);
    void on_ee_pitch_textChanged(const QString &arg1);
    void on_ee_yaw_textChanged(const QString &arg1);

    void on_enable_robot_stateChanged(int arg1);
    void on_enable_ee_stateChanged(int arg1);


    void on_create_environment_clicked();

private:
    Ui::NewScene *ui;
};

#endif // NewScene_H
