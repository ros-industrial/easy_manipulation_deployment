#ifndef LOADOBJECTS_H
#define LOADOBJECTS_H

#include <QDialog>
#include "object.h"

namespace Ui {
class LoadObjects;
}

class LoadObjects : public QDialog
{
    Q_OBJECT

public:
    bool success;
    std::vector<std::string> available_objects;
    Object chosen_object;
    std::vector<std::string> current_object_names;

    void get_all_objects();
    bool load_object_from_yaml(std::string object_name);

    explicit LoadObjects(QWidget *parent = nullptr);
    ~LoadObjects();

private slots:
    void on_ok_clicked();

    void on_exit_clicked();

    void on_available_objects_currentIndexChanged(int index);

private:
    Ui::LoadObjects *ui;
};

#endif // LOADOBJECTS_H
