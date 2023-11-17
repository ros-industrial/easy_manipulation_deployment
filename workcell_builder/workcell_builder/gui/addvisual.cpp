// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gui/addvisual.h"

#include <QFileDialog>
#include <QValidator>
#include <QKeyEvent>
#include <iostream>
#include <string>

#include "gui/ui_addvisual.h"


AddVisual::AddVisual(QWidget * parent)
: QDialog(parent),
  ui(new Ui::AddVisual)
{
  editing_mode = false;
  ui->setupUi(this);
  ui->stl_select->toggle();
  ui->TextureFIle->toggle();
  AddVisual::on_includematerial_stateChanged(0);
  AddVisual::on_includeorigin_stateChanged(0);
  success = false;
  visual.name = "None";
  visual.is_origin = false;
  visual.is_material = false;
}

AddVisual::~AddVisual()
{
  delete ui;
}

void AddVisual::on_MeshFIle_clicked()
{
  QString mesh_file = QFileDialog::getOpenFileName(this, "Open mesh filepath", QDir::homePath());
  ui->geometry_filename->setText(mesh_file);
  visual.geometry.filepath = mesh_file.toStdString();
}

void AddVisual::on_stl_select_toggled(bool checked)
{
  if (checked) {  // Use STL
    visual.geometry.disableShape();
    ui->MeshFIle->setDisabled(false);
    ui->geometry_filename->setDisabled(false);

    ui->scale_x->setDisabled(false);
    ui->scale_y->setDisabled(false);
    ui->scale_z->setDisabled(false);

    ui->scale_x_label->setDisabled(false);
    ui->scale_y_label->setDisabled(false);
    ui->scale_z_label->setDisabled(false);

    ui->comboBox->setDisabled(true);
    ui->parameter3->setDisabled(true);
    ui->parameter3_label->setDisabled(true);

    ui->parameter2->setDisabled(true);
    ui->parameter2_label->setDisabled(true);

    ui->parameter1->setDisabled(true);
    ui->parameter1_label->setDisabled(true);
  } else {  // Use Shape
    visual.geometry.disableSTL();
    ui->MeshFIle->setDisabled(true);
    ui->geometry_filename->setDisabled(true);
    ui->scale_x->setDisabled(true);
    ui->scale_y->setDisabled(true);
    ui->scale_z->setDisabled(true);

    ui->scale_x_label->setDisabled(true);
    ui->scale_y_label->setDisabled(true);
    ui->scale_z_label->setDisabled(true);
    ui->comboBox->setDisabled(false);
    ui->comboBox->setCurrentIndex(0);
    // AddVisual::on_comboBox_currentIndexChanged(ui->comboBox->currentIndex());
    this->on_comboBox_currentIndexChanged(0);
  }
}

void AddVisual::on_comboBox_currentIndexChanged(int index)
{
  if (index == 0) {  // Box
    visual.geometry.shape = "Box";
    visual.geometry.radius = -1;

    ui->parameter3->setDisabled(false);
    ui->parameter3_label->setText("Height");
    ui->parameter3_label->show();
    ui->parameter3_label->setDisabled(false);

    ui->parameter2->setDisabled(false);
    ui->parameter2_label->setText("Breadth");
    ui->parameter2_label->show();
    ui->parameter2_label->setDisabled(false);

    ui->parameter1->setDisabled(false);
    ui->parameter1_label->setText("Length");
    ui->parameter1_label->show();
    ui->parameter1_label->setDisabled(false);
  } else if (index == 1) {  // Cylinder
    visual.geometry.shape = "Cylinder";
    visual.geometry.length = -1;
    visual.geometry.breadth = -1;

    ui->parameter3->setDisabled(true);
    ui->parameter3_label->hide();

    ui->parameter2->setDisabled(false);
    ui->parameter2_label->setText("Radius");
    ui->parameter2_label->show();
    ui->parameter2_label->setDisabled(false);

    ui->parameter1->setDisabled(false);
    ui->parameter1_label->setText("Height");
    ui->parameter1_label->show();
    ui->parameter1_label->setDisabled(false);
  } else if (index == 2) {  // Circle
    visual.geometry.shape = "Sphere";
    visual.geometry.length = -1;
    visual.geometry.breadth = -1;
    visual.geometry.height = -1;

    ui->parameter3->setDisabled(true);
    ui->parameter3_label->hide();

    ui->parameter2->setDisabled(true);
    ui->parameter2_label->hide();

    ui->parameter1->setDisabled(false);
    ui->parameter1_label->setText("Radius");
  }
}

void AddVisual::on_UseTexture_toggled(bool checked)
{
  // Using texture
  if (checked) {
    visual.material.disableColor();
    ui->R->setDisabled(true);
    ui->G->setDisabled(true);
    ui->B->setDisabled(true);
    ui->A->setDisabled(true);
    ui->R_label->setDisabled(true);
    ui->G_label->setDisabled(true);
    ui->B_label->setDisabled(true);
    ui->A_label->setDisabled(true);
    ui->TextureFIle->setDisabled(false);
    ui->texture_filename->setDisabled(false);
  } else {  // Using Color
    visual.material.disableTexture();
    visual.material.is_texture = false;
    visual.material.filepath = "None";

    ui->R->setDisabled(false);
    ui->G->setDisabled(false);
    ui->B->setDisabled(false);
    ui->A->setDisabled(false);
    ui->Material_name->setDisabled(false);
    ui->R_label->setDisabled(false);
    ui->G_label->setDisabled(false);
    ui->B_label->setDisabled(false);
    ui->A_label->setDisabled(false);
    ui->Material_name_label->setDisabled(false);
    ui->TextureFIle->setDisabled(true);
    ui->texture_filename->setDisabled(true);
  }
}

void AddVisual::on_includematerial_stateChanged(int arg1)
{
  if (arg1 == 0) {  // No material
    visual.material.is_material = false;
    visual.material.disableColor();
    visual.material.disableTexture();

    ui->R->setDisabled(true);
    ui->G->setDisabled(true);
    ui->B->setDisabled(true);
    ui->A->setDisabled(true);
    ui->Material_name->setDisabled(true);

    ui->UseColor->setDisabled(true);
    ui->UseTexture->setDisabled(true);

    ui->TextureFIle->setDisabled(true);
    ui->texture_filename->setDisabled(true);

    ui->Material_name_label->setDisabled(true);
    ui->R_label->setDisabled(true);
    ui->G_label->setDisabled(true);
    ui->B_label->setDisabled(true);
    ui->A_label->setDisabled(true);
    ui->Material_title_label->setDisabled(true);
    visual.is_material = false;
  } else {  // Use Material
    visual.material.is_material = true;
    ui->Material_name->setDisabled(false);
    ui->Material_name_label->setDisabled(false);

    ui->UseColor->setDisabled(false);
    ui->UseTexture->setDisabled(false);

    ui->Material_title_label->setDisabled(false);
    visual.is_material = true;
    ui->UseTexture->setChecked(true);
  }
}

void AddVisual::on_includeorigin_stateChanged(int arg1)
{
  if (arg1 == 0) {  // No Origin
    visual.origin.disableOrigin();

    ui->x->setDisabled(true);
    ui->x_label->setDisabled(true);
    ui->y->setDisabled(true);
    ui->y_label->setDisabled(true);
    ui->z->setDisabled(true);
    ui->z_label->setDisabled(true);
    ui->roll->setDisabled(true);
    ui->roll_label->setDisabled(true);
    ui->pitch->setDisabled(true);
    ui->pitch_label->setDisabled(true);
    ui->yaw->setDisabled(true);
    ui->yaw_label->setDisabled(true);
    ui->origin_label->setDisabled(true);
    ui->position_label->setDisabled(true);
    ui->orientation_label->setDisabled(true);

    visual.is_origin = false;
  } else {  // Have origin
    visual.origin.is_origin = true;

    ui->x->setDisabled(false);
    ui->x_label->setDisabled(false);
    ui->y->setDisabled(false);
    ui->y_label->setDisabled(false);
    ui->z->setDisabled(false);
    ui->z_label->setDisabled(false);
    ui->roll->setDisabled(false);
    ui->roll_label->setDisabled(false);
    ui->pitch->setDisabled(false);
    ui->pitch_label->setDisabled(false);
    ui->yaw->setDisabled(false);
    ui->yaw_label->setDisabled(false);
    ui->origin_label->setDisabled(false);
    ui->position_label->setDisabled(false);
    ui->orientation_label->setDisabled(false);

    visual.is_origin = true;
  }
}

void AddVisual::on_visual_name_textChanged(const QString & arg1)
{
  visual.name = arg1.toStdString();
}

void AddVisual::on_Material_name_textChanged(const QString & arg1)
{
  visual.material.material_name = arg1.toStdString();
}

void AddVisual::on_TextureFIle_clicked()
{
  QString mesh_file = QFileDialog::getOpenFileName(this, "Open texture filepath", QDir::homePath());
  ui->texture_filename->setText(mesh_file);
  visual.material.filepath = mesh_file.toStdString();
}

int AddVisual::ErrorCheckMaterial()
{
  int num_errors = 0;
  if (visual.material.is_material) {
    if (!visual.material.is_texture) {
      bool all_empty_rgb =
        (ui->R->text().isEmpty() && ui->G->text().isEmpty() && ui->B->text().isEmpty() &&
        ui->A->text().isEmpty());
      bool all_full_rgb =
        (!ui->R->text().isEmpty() && !ui->G->text().isEmpty() && !ui->B->text().isEmpty() &&
        !ui->A->text().isEmpty());

      if (all_empty_rgb) {
        ui->errorlist->append(
          "RGB Options all empty."
          " Switch to texture option or disable material option if not required.");
        num_errors++;
      }
      if (!all_full_rgb && !all_empty_rgb) {
        ui->errorlist->append(
          "rgb values not complete. Fill in all fields,"
          " change to texture selection or uncheck material to disable option");
        num_errors++;
      } else {
        int i = 0;
        auto float_validator = new QDoubleValidator(0.0, 1.0, 5);
        QString input = ui->R->text();
        if (float_validator->validate(input, i) != QValidator::Acceptable) {
          ui->errorlist->append(
            "Material Error: Red (\"R\") value needs to be a positive float lesser than 1");
          num_errors++;
        } else {
          visual.material.r = input.toFloat();
        }

        input = ui->G->text();
        if (float_validator->validate(input, i) != QValidator::Acceptable) {
          ui->errorlist->append(
            "Material Error: Green (\"G\") value needs to be a positive float lesser than 1");
          num_errors++;
        } else {
          visual.material.g = input.toFloat();
        }

        input = ui->B->text();
        if (float_validator->validate(input, i) != QValidator::Acceptable) {
          ui->errorlist->append(
            "Material Error: Blue (\"B\") value needs to be a positive float lesser than 1");
          num_errors++;
        } else {
          visual.material.b = input.toFloat();
        }

        input = ui->A->text();
        if (float_validator->validate(input, i) != QValidator::Acceptable) {
          ui->errorlist->append(
            "Material Error: Alpha (\"A\") value needs to be a positive float lesser than 1");
          num_errors++;
        } else {
          visual.material.a = input.toFloat();
        }
      }
    } else if (visual.material.is_texture) {
      std::string filetype = "";
      for (auto it = visual.material.filepath.crbegin(); it != visual.material.filepath.crend();
        ++it)
      {
        if (*it != '.') {
          filetype = std::string(1, *it) + filetype;
        } else {
          break;
        }
      }
      if (filetype.compare("jpg") != 0 && filetype.compare("png") != 0) {
        // visual.material.filepath = "None";
        ui->errorlist->append(
          "File Type Error: Not a jpg or png file."
          " Select another file, or set a custom color for your material.");
        num_errors++;
      }
    }
  }
  return num_errors;
}

int AddVisual::ErrorCheckOrigin()
{
  int num_errors = 0;
  if (visual.origin.is_origin) {
    bool all_empty_xyz =
      (ui->x->text().isEmpty() && ui->y->text().isEmpty() && ui->z->text().isEmpty());
    bool all_full_xyz =
      (!ui->x->text().isEmpty() && !ui->y->text().isEmpty() && !ui->z->text().isEmpty());

    bool all_empty_rpy =
      (ui->roll->text().isEmpty() && ui->pitch->text().isEmpty() && ui->yaw->text().isEmpty());
    bool all_full_rpy =
      (!ui->roll->text().isEmpty() && !ui->pitch->text().isEmpty() && !ui->yaw->text().isEmpty());
    if (all_empty_rpy && all_empty_xyz) {
      ui->errorlist->append(
        "All Origin Fields are empty.Uncheck the Origin selection to disable the option.");
      num_errors++;
    } else {
      if (!all_full_xyz && !all_empty_xyz) {
        ui->errorlist->append("XYZ values not complete. Leave it all blank for default values");
        num_errors++;
      } else {
        if (all_full_xyz) {
          int i = 0;
          auto float_validator = new QDoubleValidator();
          QString input_x = ui->x->text();
          QString input_y = ui->y->text();
          QString input_z = ui->z->text();
          if (float_validator->validate(
              input_x,
              i) != QValidator::Acceptable ||
            float_validator->validate(
              input_y,
              i) != QValidator::Acceptable ||
            float_validator->validate(input_z, i) != QValidator::Acceptable)
          {
            ui->errorlist->append("Type Error: XYZ need to be floats");
            num_errors++;
          } else {
            visual.origin.x = input_x.toFloat();
            visual.origin.y = input_y.toFloat();
            visual.origin.z = input_z.toFloat();
          }
        }
      }

      if (!all_full_rpy && !all_empty_rpy) {
        ui->errorlist->append("RPY values not complete. Leave it all blank for default values");
        num_errors++;
      } else {
        if (all_full_rpy) {
          int i = 0;
          auto float_validator = new QDoubleValidator();
          QString input_roll = ui->roll->text();
          QString input_pitch = ui->pitch->text();
          QString input_yaw = ui->yaw->text();
          if (float_validator->validate(
              input_roll,
              i) != QValidator::Acceptable ||
            float_validator->validate(
              input_pitch,
              i) != QValidator::Acceptable ||
            float_validator->validate(input_yaw, i) != QValidator::Acceptable)
          {
            ui->errorlist->append("Type Error: RPY need to be floats");
            num_errors++;
          } else {
            visual.origin.roll = input_roll.toFloat();
            visual.origin.pitch = input_pitch.toFloat();
            visual.origin.yaw = input_yaw.toFloat();
          }
        }
      }
    }
  }
  return num_errors;
}

int AddVisual::ErrorCheckGeometry()
{
  int num_errors = 0;
  if (visual.geometry.is_stl) {
    std::string filetype = "";
    for (auto it = visual.geometry.filepath.crbegin(); it != visual.geometry.filepath.crend();
      ++it)
    {
      if (*it != '.') {
        filetype = std::string(1, *it) + filetype;
      } else {
        break;
      }
    }

    // get the new filepath which will replace the current stl filepath after it is copied
    std::string filename = "";
    for (auto it = visual.geometry.filepath.crbegin(); it != visual.geometry.filepath.crend();
      ++it)
    {
      if (*it != '/') {
        filename = std::string(1, *it) + filename;
      } else {
        break;
      }
    }
    visual.geometry.filepath_new = workcell_path.string() + "/assets/environment/" +
      "_description/meshes/visual/" + filename;
    if (filetype != "stl") {
      visual.geometry.filepath = "None";
      ui->errorlist->append("File Type Error: Not an stl file.");
      num_errors++;
    }
    bool all_empty_scale =
      (ui->scale_x->text().isEmpty() && ui->scale_y->text().isEmpty() &&
      ui->scale_z->text().isEmpty());
    bool all_full_scale =
      (!ui->scale_x->text().isEmpty() && !ui->scale_y->text().isEmpty() &&
      !ui->scale_z->text().isEmpty());
    if (all_empty_scale) {
      ui->errorlist->append(
        "All Scale Fields are empty."
        " Leave all fields at 1 for original STL size");
      num_errors++;
    } else {
      if (!all_full_scale) {
        ui->errorlist->append(
          "Scale Fields not complete. Leave all fields at 1 for original STL size");
        num_errors++;
      } else {
        int i = 0;
        auto float_validator = new QDoubleValidator();
        QString input_scalex = ui->scale_x->text();
        QString input_scaley = ui->scale_y->text();
        QString input_scalez = ui->scale_z->text();
        if (float_validator->validate(
            input_scalex,
            i) != QValidator::Acceptable ||
          float_validator->validate(
            input_scaley,
            i) != QValidator::Acceptable ||
          float_validator->validate(input_scalez, i) != QValidator::Acceptable)
        {
          ui->errorlist->append("Type Error: Scale need to be floats");
          num_errors++;
        } else {
          visual.geometry.scale_x = input_scalex.toFloat();
          visual.geometry.scale_y = input_scaley.toFloat();
          visual.geometry.scale_z = input_scalez.toFloat();
        }
      }
    }
  } else {
    // bool all_empty = false;
    bool all_full = false;
    QString input_1 = ui->parameter1->text();
    QString input_2 = ui->parameter2->text();
    QString input_3 = ui->parameter3->text();
    int i = 0;

    auto float_validator = new QDoubleValidator();
    if (visual.geometry.shape == "Box") {
      // all_empty = (ui->parameter1->text().isEmpty() && ui->parameter2->text().isEmpty() &&
      // ui->parameter3->text().isEmpty());
      all_full =
        (!ui->parameter1->text().isEmpty() && !ui->parameter2->text().isEmpty() &&
        !ui->parameter3->text().isEmpty());
      if (!all_full) {
        ui->errorlist->append("Fill up all required parameters");
        num_errors++;
      } else {
        if (float_validator->validate(
            input_1,
            i) != QValidator::Acceptable ||
          float_validator->validate(
            input_2,
            i) != QValidator::Acceptable ||
          float_validator->validate(input_3, i) != QValidator::Acceptable)
        {
          ui->errorlist->append("Type Error: Parameters need to be floats");
          num_errors++;
        } else {
          visual.geometry.length = input_1.toFloat();
          visual.geometry.breadth = input_2.toFloat();
          visual.geometry.height = input_3.toFloat();
        }
      }
    } else if (visual.geometry.shape == "Cylinder") {
      // all_empty = (ui->parameter1->text().isEmpty() && ui->parameter2->text().isEmpty());
      all_full = (!ui->parameter1->text().isEmpty() && !ui->parameter2->text().isEmpty());
      if (!all_full) {
        ui->errorlist->append("Fill up all required parameters");
        num_errors++;
      } else {
        if (float_validator->validate(
            input_1,
            i) != QValidator::Acceptable ||
          float_validator->validate(input_2, i) != QValidator::Acceptable)
        {
          ui->errorlist->append("Type Error: Parameters need to be floats");
          num_errors++;
        } else {
          visual.geometry.height = input_1.toFloat();
          visual.geometry.radius = input_2.toFloat();
        }
      }
    } else if (visual.geometry.shape == "Sphere") {
      // all_empty = (ui->parameter1->text().isEmpty());
      all_full = (!ui->parameter1->text().isEmpty());
      if (!all_full) {
        ui->errorlist->append("Fill up all required parameters");
        num_errors++;
      } else {
        if (float_validator->validate(input_1, i) != QValidator::Acceptable) {
          ui->errorlist->append("Type Error: Parameters need to be floats");
          num_errors++;
        } else {
          visual.geometry.radius = input_1.toFloat();
        }
      }
    }
  }
  return num_errors;
}

int AddVisual::ErrorVisualName()
{
  int num_errors = 0;
  if (!editing_mode) {
    for (std::string name : available_visuals) {
      if (name.compare(visual.name) == 0) {
        ui->errorlist->append("Name Error: Visual link of that name exists ");
        num_errors++;
      }
    }
  }

  if (visual.name.compare("cylinder") == 0 || visual.name.compare("box") == 0 ||
    visual.name.compare("sphere") == 0)
  {
    ui->errorlist->append(
      QString::fromStdString(
        "Name Error: " + visual.name +
        " is a reserved name. Please use another name for your visual "));
    return true;
  }
  return num_errors;
}

void AddVisual::on_ok_clicked()
{
  ui->errorlist->clear();

  int origin_errors = ErrorCheckOrigin();
  int material_errors = ErrorCheckMaterial();
  int geometry_errors = ErrorCheckGeometry();
  int name_errors = ErrorVisualName();
  if (origin_errors + material_errors + geometry_errors + name_errors == 0) {
    available_visuals.push_back(visual.name);
    success = true;
    this->close();
  }
}

void AddVisual::on_exit_clicked()
{
  success = false;
  this->close();
}

void AddVisual::load_visual(Visual visual)
{
  editing_mode = true;
  success = true;
  visual.name = visual.name;
  visual.origin = visual.origin;
  visual.geometry = visual.geometry;
  visual.material = visual.material;
  visual.is_origin = visual.is_origin;
  visual.is_material = visual.is_material;

  if (visual.name != "None") {
    ui->visual_name->setText(QString::fromStdString(visual.name));
  }
  if (visual.origin.is_origin) {
    ui->includeorigin->setChecked(true);
    on_includeorigin_stateChanged(2);
    if (visual.origin.x >= 0) {ui->x->setText(QString::number(visual.origin.x));}
    if (visual.origin.y >= 0) {ui->y->setText(QString::number(visual.origin.y));}
    if (visual.origin.z >= 0) {ui->z->setText(QString::number(visual.origin.z));}
    if (visual.origin.roll >= 0) {ui->roll->setText(QString::number(visual.origin.roll));}
    if (visual.origin.pitch >= 0) {ui->pitch->setText(QString::number(visual.origin.pitch));}
    if (visual.origin.yaw >= 0) {ui->yaw->setText(QString::number(visual.origin.yaw));}
  } else {
    on_includeorigin_stateChanged(0);
  }

  if (visual.material.is_material) {
    ui->includematerial->setChecked(true);
    on_includematerial_stateChanged(2);
    if (visual.material.material_name != "None") {
      ui->Material_name->setText(QString::fromStdString(visual.material.material_name));
    }
    if (visual.material.is_texture) {
      ui->UseTexture->toggle();
      on_UseTexture_toggled(true);
      ui->texture_filename->setText(QString::fromStdString(visual.material.filepath));
    } else {
      ui->UseColor->toggle();
      if (visual.material.r >= 0) {ui->R->setText(QString::number(visual.material.r));}
      if (visual.material.g >= 0) {ui->G->setText(QString::number(visual.material.g));}
      if (visual.material.b >= 0) {ui->B->setText(QString::number(visual.material.b));}
      if (visual.material.a >= 0) {ui->A->setText(QString::number(visual.material.a));}
    }
  } else {
    on_includematerial_stateChanged(0);
  }

  if (visual.geometry.is_stl) {
    ui->stl_select->toggle();
    on_stl_select_toggled(true);
    ui->geometry_filename->setText(QString::fromStdString(visual.geometry.filepath));
    ui->scale_x->setText(QString::number(visual.geometry.scale_x));
    ui->scale_y->setText(QString::number(visual.geometry.scale_y));
    ui->scale_z->setText(QString::number(visual.geometry.scale_z));

  } else {
    ui->geometry_select->toggle();
    on_stl_select_toggled(false);
    if (visual.geometry.shape == "Box") {
      ui->comboBox->setCurrentIndex(0);
      on_comboBox_currentIndexChanged(0);
      ui->parameter1->setText(QString::number(visual.geometry.length));
      ui->parameter2->setText(QString::number(visual.geometry.breadth));
      ui->parameter3->setText(QString::number(visual.geometry.height));
    } else if (visual.geometry.shape == "Cylinder") {
      ui->comboBox->setCurrentIndex(1);
      on_comboBox_currentIndexChanged(1);
      ui->parameter1->setText(QString::number(visual.geometry.height));
      ui->parameter2->setText(QString::number(visual.geometry.radius));
    } else if (visual.geometry.shape == "Sphere") {
      ui->comboBox->setCurrentIndex(2);
      on_comboBox_currentIndexChanged(2);
      ui->parameter1->setText(QString::number(visual.geometry.radius));
    }
  }
}

void AddVisual::keyPressEvent(QKeyEvent * e)
{
  if (e->key() != Qt::Key_Escape) {
    QDialog::keyPressEvent(e);
  } else { /* minimize */}
}
