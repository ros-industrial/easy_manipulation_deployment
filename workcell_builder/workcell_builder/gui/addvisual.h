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

#ifndef EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDVISUAL_H_
#define EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDVISUAL_H_

#include <QDialog>
#include <string>
#include <vector>

#include "boost/filesystem.hpp"
#include "attributes/visual.h"


namespace Ui
{
  class AddVisual; // NOLINT
}

class AddVisual: public QDialog
{
  Q_OBJECT
  friend class TestVisual;

public:
  bool editing_mode;
  std::vector < std::string > available_visuals;
  Visual visual;
  bool success;
  bool is_error;
  boost::filesystem::path workcell_path;
  void load_visual(Visual visual);

  int ErrorCheckMaterial();
  int ErrorCheckOrigin();
  int ErrorCheckGeometry();
  int ErrorVisualName();

  explicit AddVisual(QWidget * parent = nullptr);
  ~AddVisual();

private slots:
  void on_MeshFIle_clicked();
  void on_comboBox_currentIndexChanged(int index);
  void on_stl_select_toggled(bool checked);
  void on_UseTexture_toggled(bool checked);
  void on_includematerial_stateChanged(int arg1);
  void on_includeorigin_stateChanged(int arg1);
  void on_visual_name_textChanged(const QString & arg1);
  void on_Material_name_textChanged(const QString & arg1);
  void on_TextureFIle_clicked();
  void on_ok_clicked();
  void on_exit_clicked();
  void keyPressEvent(QKeyEvent * e);

public:
  Ui::AddVisual * ui;
};

#endif  // EASY_MANIPULATION_DEPLOYMENT__WORKCELL_BUILDER__WORKCELL_BUILDER__GUI__ADDVISUAL_H_
