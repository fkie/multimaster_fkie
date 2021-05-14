# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Fraunhofer nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# unicode_literals are not included to avoid problems with publish to ROS topics

from python_qt_binding.QtCore import Qt, Signal, QPoint, QSize
from python_qt_binding.QtGui import QBrush, QColor, QIcon, QPalette
try:
    import xmlrpclib as xmlrpcclient
except ImportError:
    import xmlrpc.client as xmlrpcclient

import os
import roslib.msgs
import roslib.names
import rospy
import ruamel.yaml
import sys
import threading
import traceback

from fkie_node_manager_daemon.common import utf8
from fkie_node_manager.detailed_msg_box import MessageBox
from fkie_node_manager.editor.line_edit import EnhancedLineEdit
from fkie_node_manager.parameter_handler import ParameterHandler

import fkie_node_manager as nm
try:
    from python_qt_binding.QtGui import QApplication, QComboBox, QCheckBox, QLineEdit, QScrollArea, QWidget
    from python_qt_binding.QtGui import QFormLayout, QHBoxLayout, QVBoxLayout, QSpacerItem, QSizePolicy
    from python_qt_binding.QtGui import QFrame, QDialog, QDialogButtonBox, QFileDialog, QLabel, QPushButton, QTextEdit
except Exception:
    from python_qt_binding.QtWidgets import QApplication, QComboBox, QCheckBox, QLineEdit, QScrollArea, QWidget
    from python_qt_binding.QtWidgets import QFormLayout, QHBoxLayout, QVBoxLayout, QSpacerItem, QSizePolicy
    from python_qt_binding.QtWidgets import QFrame, QDialog, QDialogButtonBox, QFileDialog, QLabel, QPushButton, QTextEdit


def str2bool(val):
    return val.lower() in ("yes", "true", "t", "1")


class MyComboBox(QComboBox):
    '''
    Supports the remove of items by pressing Shift+Delete.
    '''

    remove_item_signal = Signal(str)

    def __init__(self, parent=None):
        QComboBox.__init__(self, parent=parent)

    def keyPressEvent(self, event):
        key_mod = QApplication.keyboardModifiers()
        if key_mod & Qt.ShiftModifier and (event.key() == Qt.Key_Delete):
            try:
                curr_text = self.currentText()
                if curr_text:
                    for i in range(self.count()):
                        if curr_text == self.itemText(i):
                            self.removeItem(i)
                            self.remove_item_signal.emit(curr_text)
                            self.clearEditText()
            except Exception:
                print(traceback.format_exc(1))
        QComboBox.keyPressEvent(self, event)


class ValueWidget(QWidget):
    '''
    '''

    def __init__(self, parameter_description, parent=None):
        QWidget.__init__(self, parent=parent)
        self.parameter_description = parameter_description
        self._value_widget = None
        self.warn_label = QLabel(parent=self)
        self.warn_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.help_label = QLabel(parameter_description.hint, parent=self)
        self.help_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        vw = QWidget(self)
        hlayout = QHBoxLayout(vw)
        hlayout.setContentsMargins(0, 0, 0, 0)
        hlayout.addWidget(self._create_input_widget())
        if parameter_description.hint:
            # add help button if hint is available
            self.help_button = QPushButton(nm.settings().icon('info.png'), '')
            self.help_button.setFlat(True)
            self.help_button.setMaximumSize(20, 20)
            self.help_button.setCheckable(True)
            self.help_button.toggled.connect(self._on_help_toggled)
            hlayout.addWidget(self.help_button)
        vlayout = QVBoxLayout(self)
        vlayout.setContentsMargins(0, 0, 0, 0)
        vlayout.setSpacing(1)
        vlayout.addWidget(vw)
        # add label to show warnings on wrong input value
        self.warn_label.setWordWrap(True)
        vlayout.addWidget(self.warn_label)
        self.warn_label.setVisible(False)
        self.warn_label.setStyleSheet("QLabel { color: %s;}" % QColor(255, 83, 13).name())
        # help label
        self.help_label.setWordWrap(True)
        self.help_label.setStyleSheet("QLabel { background: %s;}" % QColor(255, 255, 235).name())
        vlayout.addWidget(self.help_label)
        self.help_label.setVisible(False)

    def current_text(self):
        result = ''
        if isinstance(self._value_widget, QCheckBox):
            result = repr(self._value_widget.isChecked())
        elif isinstance(self._value_widget, MyComboBox):
            result = self._value_widget.currentText()
        elif isinstance(self._value_widget, QLineEdit):
            result = self._value_widget.text()
        elif isinstance(self._value_widget, QLabel):
            result = self._value_widget.text()
        return result

    def set_value(self, value):
        if isinstance(self._value_widget, QCheckBox):
            bval = value
            if not isinstance(value, bool):
                bval = str2bool(value[0] if isinstance(value, list) else value)
            self._value_widget.setChecked(bval)
        elif isinstance(self._value_widget, MyComboBox):
            self._value_widget.setEditText(', '.join([utf8(v) for v in value]) if isinstance(value, list) else utf8(value))
        elif isinstance(self._value_widget, QLabel):
            self._value_widget.setText(value)
        elif isinstance(self._value_widget, QLineEdit):
            # avoid ' or " that escapes the string values
            self._value_widget.setText(', '.join([utf8(v) for v in value]) if isinstance(value, list) else utf8(value))

    def add_cached_values(self):
        if isinstance(self._value_widget, MyComboBox):
            fullname = self.parameter_description.fullName()
            values = nm.history().cachedParamValues(fullname)
            for i in range(self._value_widget.count()):
                try:
                    values.remove(self._value_widget.itemText(i))
                except ValueError:
                    pass
                except Exception:
                    print(traceback.format_exc())
            if self._value_widget.count() == 0:
                values.insert(0, '')
            self._value_widget.addItems(values)

    def _create_input_widget(self):
        pd = self.parameter_description
        value = pd._value
        if 'bool' in pd.baseType():
            # add checkbox to edit boolean value
            cb = QCheckBox(parent=self)
            cb.setObjectName(pd.name())
            if not isinstance(value, bool):
                value = str2bool(value[0] if isinstance(value, list) else value)
            pd._value_org = value
            cb.setChecked(value)
            cb.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
            cb.setMinimumHeight(20)
            self._value_widget = cb
            return cb
        elif pd.read_only:
            # read only value are added as label
            label = QLabel(value, parent=self)
            label.setMinimumHeight(20)
            label.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self._value_widget = label
            return label
        else:
            # all other are added as combobox
            cb = MyComboBox(parent=self)
            cb.setObjectName(pd.name())
            cb.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
            cb.setEditable(True)
            cb.remove_item_signal.connect(pd.removeCachedValue)
            cb.editTextChanged.connect(self._check_text)
            items = []
            if isinstance(value, list):
                if pd.isArrayType():
                    items.append(','.join([utf8(val) for val in value]))
                else:
                    items[len(items):] = value
            else:
                if value is not None and value:
                    items.append(utf8(value) if not isinstance(value, xmlrpcclient.Binary) else '{binary data!!! updates will be ignored!!!}')
                elif pd.isTimeType():
                    items.append('now')
            if ':alt' in pd._tags:
                # add alternative values
                try:
                    for alt_value in pd._tags[':alt']:
                        if alt_value not in items:
                            items.append(alt_value)
                except Exception as err:
                    rospy.logwarn('Can not add alternative values to %s: %s' % (pd.name(), utf8(err)))
            pd._value_org = items[0] if items else ''
            cb.addItems(items)
            self._value_widget = cb
            if pd.path_type:
                # add path editor if path type is defined
                fd = QWidget(self)
                hlayout = QHBoxLayout(fd)
                hlayout.setContentsMargins(0, 0, 0, 0)
                hlayout.addWidget(cb)
                self.path_button = QPushButton('...')
                self.path_button.setFlat(True)
                self.path_button.setMaximumSize(20, 20)
                self.path_button.clicked.connect(self._on_file_dialog_clicked)
                hlayout.addWidget(self.path_button)
                return fd
            else:
                return cb

    def _check_text(self, text=''):
        '''
        Checks the content of the combobox for valid type
        '''
        try:
            self.parameter_description.updateValue(text)
            # self.combobox.setStyleSheet('')
            self.warn_label.setVisible(False)
        except Exception as err:
            self.warn_label.setText(utf8(err))
            # bg_style = "MyComboBox { background: %s;}" % QColor(255, 83, 13).name()
            # self.combobox.setStyleSheet("%s" % (bg_style))
            self.warn_label.setVisible(True)

    def _on_file_dialog_clicked(self):
        # Workaround for QFileDialog.getExistingDirectory because it do not
        # select the configuration folder in the dialog
        self.dialog = QFileDialog(self, caption=self.parameter_description.hint)
        self.dialog.setOption(QFileDialog.HideNameFilterDetails, True)
        if self.parameter_description.path_type == 'dir':
            self.dialog.setFileMode(QFileDialog.Directory)
        self.dialog.setDirectory(self._value_widget.currentText())
        if self.dialog.exec_():
            fileNames = self.dialog.selectedFiles()
            self._value_widget.setEditText(fileNames[0])

    def _on_help_toggled(self, checked):
        self.help_label.setVisible(checked)


class ParameterDescription(object):
    '''
    Used for internal representation of the parameter in dialog.
    '''

    def __init__(self, name, msg_type, value=None, widget=None):
        self._name = str(name)
        self._type = msg_type
        self._value = None
        self._value_org = None
        self.read_only = False
        self.path_type = ''
        self.hint = ''
        self._min = None
        self._max = None
        self._tags = {}
        self._read_value(value)
        self._widget = widget
        try:
            self._base_type, self._is_array_type, self._array_length = roslib.msgs.parse_type(self._type)
        except Exception:
            pass
        if msg_type == 'binary':
            self._base_type = msg_type

    def _read_value(self, value):
        if isinstance(value, dict):
            for key, val in value.items():
                if key.startswith(':'):
                    if key == ':value':
                        self._value = val
                        self._value_org = val
                    elif key == ':ro':
                        self.read_only = val
                    elif key == ':hint':
                        self.hint = val
                    elif key == ':path':
                        self.path_type = val
                    elif key == ':min':
                        self._min = val
                    elif key == ':max':
                        self._max = val
                    self._tags[key] = val
        else:
            self._value = value
            self._value_org = value

    def __repr__(self):
        return "%s [%s]: %s" % (self._name, self._type, utf8(self._value))

    @classmethod
    def is_primitive_type(cls, value_type):
        result = value_type in roslib.msgs.PRIMITIVE_TYPES
        result = result or value_type in ['string', 'int', 'float', 'time', 'duration', 'binary', 'unicode']
        return result

    def add_tag(self, key, value):
        self._tags[key] = value

    def origin_value(self):
        return self._value_org

    def clear_origin_value(self):
        self._value_org = None

    def changed(self):
        return utf8(self.origin_value()) != utf8(self._value)

    def name(self):
        return self._name

    def setWidget(self, widget):
        self._widget = widget
        if widget is not None:
            self.addCachedValuesToWidget()

    def widget(self):
        return self._widget

    def fullName(self):
        result = self.name()
        widget = self._widget
        while widget is not None:
            if isinstance(widget, (MainBox, GroupBox, ArrayBox)):
                result = roslib.names.ns_join(widget.name, result)
            widget = widget.parent()
        return result

    def isArrayType(self):
        # handle representation of `rosparam`
        return self._is_array_type or (self._type in ['[]'])

    def arrayLength(self):
        return self._array_length

    def isPrimitiveType(self):
        result = self.is_primitive_type(self._base_type)
        # if value is a string, the list is represented as a string, see `rosparam`
        result = result or self._type in ['[]']
        return result

    def isTimeType(self):
        return self._base_type in ['time', 'duration']

    def isBinaryType(self):
        return self._base_type in ['binary']

    def baseType(self):
        return self._base_type

    def msgType(self):
        return self._type

    def updateValueFromField(self):
        if self.read_only:
            # do no change any values
            return
        result = self.widget().current_text()
        self._value = self.updateValue(result, raise_on_min_max=False)
        if self.changed():
            nm.history().addParamCache(self.fullName(), self._value)

    def updateValue(self, value, raise_on_min_max=True):
        rvalue = value
        try:
            if isinstance(value, (dict, list)):
                rvalue = value
            elif value:
                if self.isArrayType():
                    if 'int' in self.baseType() or 'byte' in self.baseType():
                        rvalue = map(int, value.lstrip('[').rstrip(']').split(','))
                    elif 'float' in self.baseType():
                        rvalue = map(float, value.lstrip('[').rstrip(']').split(','))
                    elif 'bool' in self.baseType():
                        rvalue = map(str2bool, value.lstrip('[').rstrip(']').split(','))
                    elif self.isBinaryType():
                        rvalue = value
                    else:
                        try:
                            rvalue = value.lstrip('[').rstrip(']')
                            rvalue = ruamel.yaml.load("[%s]" % rvalue, Loader=ruamel.yaml.Loader)
                            # if there is no YAML, load() will return an
                            # empty string.  We want an empty dictionary instead
                            # for our representation of empty.
                            if rvalue is None:
                                rvalue = []
                        except ruamel.yaml.MarkedYAMLError as e:
                            raise Exception("Field [%s] yaml error: %s" % (self.fullName(), utf8(e)))
                    if self.arrayLength() is not None and self.arrayLength() != len(rvalue):
                        raise Exception(''.join(["Field [", self.fullName(), "] has incorrect number of elements: ", utf8(len(rvalue)), " != ", str(self.arrayLength())]))
                else:
                    if 'int' in self.baseType() or 'byte' in self.baseType():
                        rvalue = int(value)
                    elif 'float' in self.baseType():
                        rvalue = float(value)
                    elif 'bool' in self.baseType():
                        if isinstance(value, bool):
                            rvalue = value
                        else:
                            rvalue = str2bool(value)
                    elif self.isBinaryType():
                        rvalue = utf8(value)
                    elif self.isTimeType():
                        if value == 'now':
                            rvalue = 'now'
                        else:
                            try:
                                val = eval(value)
                                if isinstance(val, dict):
                                    rvalue = val
                                else:
                                    secs = int(val)
                                    nsecs = int((val - secs) * 1000000000)
                                    rvalue = {'secs': secs, 'nsecs': nsecs}
                            except Exception:
                                rvalue = {'secs': 0, 'nsecs': 0}
                    else:
                        if sys.version_info[0] <= 2:
                            rvalue = value.encode(sys.getfilesystemencoding())
                        else:
                            rvalue = value
            else:
                if self.isArrayType():
                    arr = []
                    rvalue = arr
                else:
                    if 'int' in self.baseType() or 'byte' in self.baseType():
                        rvalue = 0
                    elif 'float' in self.baseType():
                        rvalue = 0.0
                    elif 'bool' in self.baseType():
                        rvalue = False
                    elif self.isBinaryType():
                        rvalue = utf8(value)
                    elif self.isTimeType():
                        rvalue = {'secs': 0, 'nsecs': 0}
                    else:
                        rvalue = ''
        except Exception as e:
            raise Exception("Error while set value '%s', for '%s': %s" % (utf8(value), self.fullName(), utf8(e)))
        if self._min is not None:
            if rvalue < self._min:
                if raise_on_min_max:
                    raise Exception("%s is smaller than minimum: %s" % (utf8(rvalue), utf8(self._min)))
                rvalue = self._min
        if self._max is not None:
            if rvalue > self._max:
                if raise_on_min_max:
                    raise Exception("%s is greater than maximum: %s" % (utf8(rvalue), utf8(self._max)))
                rvalue = self._max
        return rvalue

    def value(self, with_tags=False):
        if not self.isPrimitiveType() and not self.widget() is None:
            return self.widget().value(with_tags)
        elif self.isPrimitiveType():
            self.updateValueFromField()
        if with_tags:
            result = {}
            result.update(self._tags)
            result[':value'] = self._value
            return result
        return self._value

    def removeCachedValue(self, value):
        nm.history().removeParamCache(self.fullName(), value)

    def createTypedWidget(self, parent):
        result = None
        if self.isPrimitiveType():
            result = ValueWidget(self, parent)
        else:
            if self.isArrayType():
                result = ArrayBox(self.name(), self._type, dynamic=self.arrayLength() is None, parent=parent)
            else:
                result = GroupBox(self.name(), self._type, parent=parent)
        return result

    def addCachedValuesToWidget(self):
        if isinstance(self.widget(), ValueWidget):
            self.widget().add_cached_values()


class MainBox(QFrame):
    '''
    Groups the parameter without visualization of the group. It is the main widget.
    '''

    def __init__(self, name, param_type, collapsible=True, parent=None):
        QFrame.__init__(self, parent)
        self.setObjectName(name)
        self.name = name
        self.type_msg = param_type
        self.params = []
        self.collapsed = False
        self.parameter_description = None
        vLayout = QVBoxLayout(self)
        vLayout.setContentsMargins(1, 1, 1, 1)
        vLayout.setSpacing(1)
        self.param_widget = QFrame(self)
        self.collapsible = collapsible
        if collapsible:
            self.options_layout = QHBoxLayout()
            self.options_layout.setContentsMargins(1, 1, 1, 1)
            self.options_layout.setSpacing(1)
            self.hide_button = QPushButton('-')
            self.hide_button.setFlat(True)
            self.hide_button.setMaximumSize(20, 20)
            self.hide_button.clicked.connect(self._on_hide_clicked)
            self.name_label = QLabel(name)
            self.name_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            font = self.name_label.font()
            font.setBold(True)
            self.name_label.setFont(font)
            self.options_layout.addWidget(self.hide_button)
            self.options_layout.addWidget(self.name_label)
            self.type_label = QLabel('(%s)' % param_type)
            self.options_layout.addWidget(self.type_label)
            self.options_layout.addStretch()
            vLayout.addLayout(self.options_layout)
            self.param_widget.setFrameShape(QFrame.StyledPanel)
            self.param_widget.setFrameShadow(QFrame.Sunken)
        boxLayout = QFormLayout(self.param_widget)
        boxLayout.setContentsMargins(3, 3, 3, 3)
        boxLayout.setVerticalSpacing(1)
        vLayout.addWidget(self.param_widget)
        if param_type in ['std_msgs/Header']:
            self.setCollapsed(True)

    def setCollapsed(self, value):
        self.collapsed = value
        self.param_widget.setVisible(not value)
        self.hide_button.setText('+' if self.collapsed else '-')

    def _on_hide_clicked(self):
        self.setCollapsed(not self.collapsed)
#    self.param_widget.setVisible(not self.param_widget.isVisible())
#    vis = self.param_widget.isVisible()
#    self.hide_button.setText('-' if vis else '+')

    def createFieldFromValue(self, value, clear_origin_value=False):
        self.setUpdatesEnabled(False)
        try:
            if isinstance(value, (dict, list)):
                self._createFieldFromDict(value, clear_origin_value=clear_origin_value)
        except Exception:
            print(traceback.format_exc())
        finally:
            self.setUpdatesEnabled(True)

    def _createFieldFromDict(self, value, layout=None, clear_origin_value=False):
        if layout is None:
            layout = self.param_widget.layout()
        # sort the items: 1. header, 2. all primitives (sorted), 3. list, dict (sorted)
        all_params = []
        primitives = []
        komplex = []
        for name, val in value.items():
            _type = type(val).__name__
            if isinstance(val, dict):
                if ':type' in val:
                    _type = val[':type']
                elif ':value' in val:
                    _type = type(val[':value']).__name__
            if _type == 'str':
                _type = 'string'
            if _type in ['std_msgs/Header']:
                all_params.append((name, _type, val))
            elif ParameterDescription.is_primitive_type(_type):
                primitives.append((name, _type, val))
            else:
                komplex.append((name, _type, val))
        all_params.extend(sorted(primitives))
        all_params.extend(sorted(komplex))

        # create widgets
        for name, _type, val in all_params:
            if name.startswith(':'):
                continue
            # search for existing field
            field = self.getField(name)
            if field is None:
                # add parameter object first
                param_desc = ParameterDescription(name, _type, val)
                # create widget for parameter
                field = param_desc.createTypedWidget(self)
                if clear_origin_value:
                    param_desc.clear_origin_value()
                param_desc.setWidget(field)
                self.params.append(param_desc)
                if isinstance(field, (GroupBox, ArrayBox)):
                    field.createFieldFromValue(val[':value'] if ':value' in val else val, clear_origin_value)
                    layout.addRow(field)
                else:
                    # we have e simple parameter, create label for it
                    label_name = name if _type in ['string', 'str', 'unicode', 'bool'] else '%s (%s)' % (name, _type)
                    label = QLabel(label_name, self)
                    label.setObjectName('%s_label' % name)
                    label.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding))
                    label.setTextInteractionFlags(Qt.TextSelectableByMouse)
                    hint = field.toolTip()
                    if hint:
                        label.setToolTip(hint)
                        # whatsThis destroys the layout
                        # label.whatsThis(hint)
                    label.setBuddy(field)
                    layout.addRow(label, field)
            else:
                # field exists already -> update groups or arrays
                if isinstance(field, (GroupBox, ArrayBox)):
                    field.createFieldFromValue(val[':value'] if ':value' in val else val, clear_origin_value)
                else:
                    raise Exception("Parameter with name '%s' already exists!" % name)

    def value(self, with_tags=False, only_changed=False):
        result = dict()
        for param in self.params:
            if not param.isBinaryType():
                if param.isPrimitiveType():
                    val = param.value(with_tags=with_tags)
                    if param.changed() or not only_changed:
                        result[param.name()] = val
                else:
                    val = param.value(with_tags=with_tags)
                    if val or not only_changed:
                        result[param.name()] = val
        return result

    def set_values(self, values):
        '''
        Sets the values for existing fields. Used e.g. while load parameter from file

        :param dict values: the dictionary with values to set.
        :raise Exception: on errors
        '''
        if isinstance(values, dict):
            for param, val in values.items():
                value = val
                _type = 'unknown'
                if isinstance(val, tuple):
                    # backward compatibility
                    (_type, value) = val
                elif isinstance(val, dict):
                    if ':value' in val:
                        value = val[':value']
                    if ':type' in val:
                        _type = val[':type']
                field = self.getField(param)
                if field is not None:
                    if isinstance(field, (GroupBox, ArrayBox)):
                        field.set_values(value)
                    else:
                        field.set_value(value)
        elif isinstance(values, list):
            raise Exception("Setting 'list' values in MainBox or GroupBox not supported!!!")

    def getField(self, name, recursive=False):
        for child in self.children():
            for c in child.children():
                if recursive and isinstance(c, MainBox):
                    result = c.getField(name, recursive=recursive)
                    if result is not None:
                        return result
                elif c.objectName() == name:
                    return c
        return None

    def removeAllFields(self):
        '''
        Remove the references between parameter and corresponding widgets
        (ComboBox, CheckBox, ..) and remove these widgets from layouts.
        '''
        for child in self.param_widget.children():
            if isinstance(child, MyComboBox):
                child.parameter_description.setWidget(None)
                self.params.remove(child.parameter_description)
            elif isinstance(child, MainBox):
                child.removeAllFields()
                self.param_widget.layout().removeWidget(child)

    def filter(self, arg, force_show=False):
        '''
        Hide the parameter input field, which label dosn't contains the C{arg}.

        :param str arg: the filter text
        :param bool force_show: override filter, if group is shown
        '''
        result = False
        for child in self.param_widget.children():
            if isinstance(child, (MainBox, GroupBox, ArrayBox)):
                show = force_show or not arg
                if not show:
                    show = child.objectName().lower().find(arg) != -1
                show = child.filter(arg, force_show=show) or show
                # hide group, if no parameter are visible
                child.setVisible(show)
                if show:
                    child.setCollapsed(False)
                    result = True
            elif isinstance(child, ValueWidget):
                label = child.parentWidget().layout().labelForField(child)
                if label is not None:
                    show = force_show or not arg
                    if not show:
                        show = child.current_text().lower().find(arg) != -1 or label.text().lower().find(arg) != -1
                    # set the parent group visible if it is not visible
                    if show and not child.parentWidget().isVisible():
                        child.parentWidget().setVisible(show)
                    label.setVisible(show)
                    child.setVisible(show)
                    if show:
                        result = True
        return result

    def setVisible(self, arg):
        if arg and not self.parentWidget() is None and not self.parentWidget().isVisible():
            self.parentWidget().setVisible(arg)
        QWidget.setVisible(self, arg)


class GroupBox(MainBox):
    '''
    Groups the parameter of a dictionary, struct or class using the group box for
    visualization.
    '''

    def __init__(self, name, param_type, parent=None):
        MainBox.__init__(self, name, param_type, True, parent)
        self.setObjectName(name)


class ArrayEntry(MainBox):
    '''
    A part of the ArrayBox to represent the elements of a list.
    '''

    def __init__(self, index, param_type, parent=None):
        MainBox.__init__(self, '#%s' % utf8(index), param_type, True, parent)
        self.index = index
        self.setObjectName(''.join(['[', utf8(index), ']']))
        self.param_widget.setFrameShape(QFrame.Box)
        self.param_widget.setFrameShadow(QFrame.Plain)
        self.type_label.setVisible(False)
#    boxLayout = QFormLayout()
#    boxLayout.setVerticalSpacing(0)
#    label = QLabel(''.join(['[', str(index), ']']))
#    self.param_widget.layout().addRow(label)
#    self.setLayout(boxLayout)

    def value(self, with_tags=False, only_changed=False):
        '''
        Retruns a dictionary for an entry of an array, e.g. {name: value}.
        If with_tags is True it looks like: {name: {':value': value, ':type': type}}
        :rtype: dict
        '''
        result = dict()
        for param in self.params:
            val = param.value(with_tags)
            if val or not only_changed:
                result[param.name()] = val
        return result


class ArrayBox(MainBox):
    '''
    Groups the parameter of a list.
    '''

    def __init__(self, name, param_type, dynamic, parent=None):
        MainBox.__init__(self, name, param_type, True, parent)
        self._is_dynamic = dynamic
        self._dynamic_value = None
        self._dynamic_widget = None
        self._dynamic_items_count = 0

    def addDynamicBox(self):
        self._dynamic_items_count = 0
        addButton = QPushButton("+")
        addButton.setMaximumSize(25, 25)
        addButton.clicked.connect(self._on_add_dynamic_entry)
        self.options_layout.addWidget(addButton)
        self.count_label = QLabel('0')
        self.options_layout.addWidget(self.count_label)
        remButton = QPushButton("-")
        remButton.setMaximumSize(25, 25)
        remButton.clicked.connect(self._on_rem_dynamic_entry)
        self.options_layout.addWidget(remButton)

    def _on_add_dynamic_entry(self, checked=False, value=None):
        self.setUpdatesEnabled(False)
        try:
            val = value
            if val is None:
                val = self._dynamic_value
            if val is not None:
                self._create_dynamic_frame(val)
        finally:
            self.setUpdatesEnabled(True)

    def _create_dynamic_frame(self, value):
        entry_frame = ArrayEntry(self._dynamic_items_count, self.type_msg)
        self.param_widget.layout().addRow(entry_frame)
        entry_frame._createFieldFromDict(value)
        self._dynamic_items_count += 1
        self.count_label.setText(utf8(self._dynamic_items_count))

    def _on_rem_dynamic_entry(self):
        if self._dynamic_items_count > 0:
            self._dynamic_items_count -= 1
            item = self.param_widget.layout().takeAt(self._dynamic_items_count)
            self.param_widget.layout().removeItem(item)
            try:
                # remove the referenced parameter, too
                for child in item.widget().children():
                    if isinstance(child, MyComboBox):
                        child.parameter_description.setWidget(None)
                        self.params.remove(child.parameter_description)
                    elif isinstance(child, MainBox):
                        child.removeAllFields()
                        self.param_widget.layout().removeWidget(child)
                        child.parameter_description.setWidget(None)
                        self.params.remove(child.parameter_description)
                item.widget().setParent(None)
                del item
            except Exception:
                print(traceback.format_exc(3))
            self.count_label.setText(utf8(self._dynamic_items_count))

    def createFieldFromValue(self, value, clear_origin_value=False):
        self.setUpdatesEnabled(False)
        try:
            if self._is_dynamic:
                self.addDynamicBox()
                # Set value used to add dynamic array fields.
                # On republish there is an array filled array. So only last enry will be used on add new entry.
                if isinstance(value, list):
                    if value:
                        self._dynamic_value = value[-1]
                else:
                    self._dynamic_value = value
                self.set_values(value)
        except Exception:
            print(traceback.format_exc())
        finally:
            self.setUpdatesEnabled(True)

    def value(self, with_tags=False, only_changed=False):
        '''
        Goes through the list and creates dictionary with values of each element.
        Returns a list with dictionaries, e.g. [{name: value}, {name: value}].
        If with_tags is True the result is a dictionary, e.g. {':type': type[], ':value': [{name: value}, {name: value}]}

        :rtype: list or dict, if with_tags==True
        '''
        result_list = list()
        for i in range(self.param_widget.layout().rowCount()):
            item = self.param_widget.layout().itemAt(i, QFormLayout.SpanningRole)
            if item and isinstance(item.widget(), ArrayEntry):
                value = item.widget().value(with_tags=with_tags, only_changed=only_changed)
                result_list.append(value)
        result = result_list
        if with_tags:
            result = {}
            result[':type'] = self.type_msg
            result[':value'] = result_list
        return result

    def set_values(self, values):
        '''
        Create a list of the elements and sets their values.

        :param list values: The list of dictionaries with parameter values
        '''
        if isinstance(values, list):
            count_entries = 0
            # determine the count of existing elements
            for i in range(self.param_widget.layout().rowCount()):
                item = self.param_widget.layout().itemAt(i, QFormLayout.SpanningRole)
                if item and isinstance(item.widget(), ArrayEntry):
                    count_entries += 1
            # create the list of the elements of the length of values
            if count_entries < len(values):
                for i in range(len(values) - count_entries):
                    # use array entry
                    self._on_add_dynamic_entry(value=values[i])
            elif count_entries > len(values):
                for i in range(count_entries - len(values)):
                    self._on_rem_dynamic_entry()
            # set the values
            for i in range(self.param_widget.layout().rowCount()):
                item = self.param_widget.layout().itemAt(i, QFormLayout.SpanningRole)
                if item and isinstance(item.widget(), ArrayEntry):
                    item.widget().set_values(values[i])


class ScrollArea(QScrollArea):
    '''
    ScrollArea provides the maximal width of the internal widget.
    '''

    def viewportEvent(self, arg):
        if self.widget() and self.viewport().size().width() != self.widget().maximumWidth():
            self.widget().setMaximumWidth(self.viewport().size().width())
        return QScrollArea.viewportEvent(self, arg)


class ParameterDialog(QDialog):
    '''
    This dialog creates an input mask for the given parameter and their types.
    '''

    def __init__(self, params=dict(), buttons=QDialogButtonBox.Cancel | QDialogButtonBox.Ok, sidebar_var='', parent=None, store_geometry=''):
        '''
        Creates an input dialog.

        :param dict params: a (recursive) dictionary with parameter names and their values.
        A value can be of primitive type (int, bool, string), a list or dictionary. If it is
        of list type, the list should contains dictionaries with parameter and values.
        If value is of dictionary type it is a recursive include or value with tags.
        If it is a recursive include a group will be created. The key is the name of the group.
        If it is a value with tags it should contains at least a ':value' tag.
        All attributes begin with ':'. Other key attributes:
        -':type': type, overwrites the autodetection
        -':ro': read only
        -':hint': description of the parameter
        -':default': default value
        -':min': minimum value
        -':max': maximum value
        -':alt': a list of alternative values
        -'path': 'dir' or 'file'
        :param str sidebar_var: the name of the key in first level of params. Creates a sidebar if
        it is not empty. Cached and alternative values are used to fill the sidebar.
        '''
        QDialog.__init__(self, parent=parent)
        self.setObjectName('ParameterDialog - %s' % utf8(params))

        self.__current_path = nm.settings().current_dialog_path
        self.horizontalLayout = QHBoxLayout(self)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setSpacing(0)
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout.setContentsMargins(3, 3, 3, 3)
        # add filter row
        self.filter_field = EnhancedLineEdit(self)
        self.filter_field.setPlaceholderText("filter")
        self.filter_field.textChanged.connect(self._on_filter_changed)
        self.filter_visible = True

        self.verticalLayout.addWidget(self.filter_field)

        # create area for the parameter
        self.scrollArea = scrollArea = ScrollArea(self)
        scrollArea.setObjectName("scrollArea")
        self.content = MainBox('/', 'string', False, self)
        scrollArea.setFrameStyle(QFrame.NoFrame)
        scrollArea.setWidget(self.content)
        scrollArea.setWidgetResizable(True)
        self.verticalLayout.addWidget(scrollArea)

        # add info text field
        self.info_field = QTextEdit(self)
        palette = QPalette()
        brush = QBrush(QColor(255, 254, 242))
        brush.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.Base, brush)
        brush = QBrush(QColor(255, 254, 242))
        brush.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Inactive, QPalette.Base, brush)
        brush = QBrush(QColor(244, 244, 244))
        brush.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Disabled, QPalette.Base, brush)
        self.info_field.setPalette(palette)
        self.info_field.setFrameShadow(QFrame.Plain)
        self.info_field.setReadOnly(True)
        self.info_field.setTextInteractionFlags(Qt.LinksAccessibleByKeyboard | Qt.LinksAccessibleByMouse | Qt.TextBrowserInteraction | Qt.TextSelectableByKeyboard | Qt.TextSelectableByMouse)
        self.info_field.setObjectName("dialog_info_field")
        self.verticalLayout.addWidget(self.info_field)
        self.info_field.setVisible(False)

        # create buttons
        self.buttonBox = QDialogButtonBox(self)
        self.buttonBox.setObjectName("buttonBox")
        self.buttonBox.setOrientation(Qt.Horizontal)
        self.buttonBox.setStandardButtons(buttons)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
        self.verticalLayout.addWidget(self.buttonBox)
        self.horizontalLayout.addLayout(self.verticalLayout)

        # add side bar for checklist
        values = nm.history().cachedParamValues('/%s' % sidebar_var)
        self.sidebar_frame = QFrame(self)
        self.sidebar_frame.setObjectName(sidebar_var)
        sidebarframe_verticalLayout = QVBoxLayout(self.sidebar_frame)
        sidebarframe_verticalLayout.setObjectName("sidebarframe_verticalLayout")
        sidebarframe_verticalLayout.setContentsMargins(3, 3, 3, 3)
        self._sidebar_selected = 0
        if len(values) > 0 and sidebar_var in params:
            self.horizontalLayout.addWidget(self.sidebar_frame)
            try:
                if ':value' in params[sidebar_var]:
                    self.sidebar_default_val = params[sidebar_var][':value']
                else:
                    self.sidebar_default_val = params[sidebar_var][1]
                # add default value to sidebar
                if self.sidebar_default_val and self.sidebar_default_val not in values:
                    values.append(self.sidebar_default_val)
            except Exception:
                self.sidebar_default_val = ''
            values.sort()
            for v in values:
                checkbox = QCheckBox(v)
                checkbox.setObjectName(v)
                checkbox.stateChanged.connect(self._on_sidebar_stateChanged)
                self.sidebar_frame.layout().addWidget(checkbox)
            self.sidebar_frame.layout().addItem(QSpacerItem(100, 20, QSizePolicy.Minimum, QSizePolicy.Expanding))
        # set the input fields
        if params:
            try:
                self.content.createFieldFromValue(params)
                self.setInfoActive(False)
            except Exception:
                print(traceback.format_exc())

        if self.filter_field.isVisible():
            self.filter_field.setFocus()
        # restore from configuration file
        self._geometry_name = store_geometry
        if store_geometry and nm.settings().store_geometry:
            settings = nm.settings().qsettings(nm.settings().CFG_GUI_FILE)
            self._history_selected_robot = settings.value("selected_robot", '')
            settings.beginGroup(store_geometry)
            self.resize(settings.value("size", QSize(600, 300)))
            pos = settings.value("pos", QPoint(0, 0))
            if pos.x() != 0 and pos.y() != 0:
                self.move(pos)
            settings.endGroup()

    def __del__(self):
        self.content.removeAllFields()

    def _on_sidebar_stateChanged(self, state):
        if state == Qt.Checked:
            self._sidebar_selected += 1
        elif state == Qt.Unchecked:
            self._sidebar_selected -= 1
        if self._sidebar_selected in [0, 1]:
            try:
                field = self.content.getField(self.sidebar_frame.objectName())
                if field is not None and field.currentText() == self.sidebar_default_val:
                    field.setEnabled(True if self._sidebar_selected == 0 else False)
            except Exception:
                pass

    def showLoadSaveButtons(self):
        self.load_button = QPushButton()
        self.load_button.setIcon(nm.settings().icon('load.png'))
        self.load_button.clicked.connect(self._load_parameter)
        self.load_button.setToolTip('Load parameters from YAML file')
        self.load_button.setFlat(True)
        self.buttonBox.addButton(self.load_button, QDialogButtonBox.ActionRole)
        self.save_button = QPushButton()
        self.save_button.clicked.connect(self._save_parameter)
        self.save_button.setIcon(nm.settings().icon('save.png'))
        self.save_button.setToolTip('Save parameters to YAML file')
        self.save_button.setFlat(True)
        self.buttonBox.addButton(self.save_button, QDialogButtonBox.ActionRole)

    def _on_filter_changed(self):
        self.content.filter(self.filter_field.text().lower())

    def setFilterVisible(self, val):
        '''
        Shows or hides the filter row.
        '''
        self.filter_visible = val
        self.filter_field.setVisible(val & self.scrollArea.isHidden())

    def add_warning(self, message):
        label = QLabel(self)
        label.setWordWrap(True)
        label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        label.setText(''.join(["<font color='red'>Warning!\n", message, "</font>"]))
        self.verticalLayout.insertWidget(1, label)

    def setText(self, text):
        '''
        Adds a label to the dialog's layout and shows the given text.

        :param str text: the text to add to the dialog
        '''
        self.info_field.setText(text)
        self.setInfoActive(True)

    def setInfoActive(self, val):
        '''
        Activates or deactivates the info field of this dialog. If info field is
        activated, the filter frame and the input field are deactivated.

        :param bool val: state
        '''
        if val and self.info_field.isHidden():
            self.filter_field.setVisible(False & self.filter_visible)
            self.scrollArea.setVisible(False)
            self.info_field.setVisible(True)
        elif not val and self.scrollArea.isHidden():
            self.filter_field.setVisible(True & self.filter_visible)
            self.scrollArea.setVisible(True)
            self.info_field.setVisible(False)
            if self.filter_field.isVisible():
                self.filter_field.setFocus()

    def setFocusField(self, field_label):
        field = self.content.getField(field_label, recursive=True)
        if field is not None:
            field.setFocus()

    def getKeywords(self, only_changed=False, with_tags=False):
        '''
        :param bool only_changed: returns changed parameter only (Defaul: False)
        :param bool with_tags: returns parameter attributes (e.g. :ro, :hint,...) (Defaul: False)
        :returns  a directory with parameter and value for entered fields.
        :rtype: dict
        '''
        # get the results of sidebar
        sidebar_list = []
        sidebar_name = self.sidebar_frame.objectName()
        for j in range(self.sidebar_frame.layout().count() - 1):
            w = self.sidebar_frame.layout().itemAt(j).widget()
            if isinstance(w, QCheckBox):
                if w.checkState() == Qt.Checked:
                    sidebar_list.append(w.objectName())
        result_value = self.content.value(with_tags, only_changed)
        # add the sidebar results
        if sidebar_name in result_value:
            # skip the default value, if elements are selected in the side_bar
            sidebar_value = ''
            if with_tags:
                sidebar_value = result_value[sidebar_name][':value']
            else:
                sidebar_value = result_value[sidebar_name]
            if len(sidebar_list) == 0 or self.sidebar_default_val != sidebar_value:
                sidebar_list.append(sidebar_value)
            if with_tags:
                result_value[sidebar_name][':value'] = [v for v in set(sidebar_list)]
            else:
                result_value[sidebar_name] = [v for v in set(sidebar_list)]
        return result_value

    def keywords2params(self, keywords):
        '''
        Resolves the dictionary values to ROS parameter names.

        :param keywords: the result of the getKeywords
        :return: dictionary of (ROS parameter name : value)
        '''
        result = dict()
        for param, value in keywords.items():
            if isinstance(value, dict):
                r = self.keywords2params(value)
                for p, v in r.items():
                    result[roslib.names.ns_join(param, p)] = v
            else:
                result[param] = value
        return result

    @classmethod
    def remove_attributes(cls, keywords):
        # it it is a value dictionary, we need only :value attribute
        if ':value' in keywords:
            return keywords[':value']
        # remove all attributes which starts with ':'
        result = {}
        for key, val in keywords.items():
            clean_val = val
            if isinstance(val, dict):
                clean_val = cls.remove_attributes(val)
            if not key.startswith(':'):
                result[key] = clean_val
        return result

    def _save_parameter(self):
        try:
            (fileName, _) = QFileDialog.getSaveFileName(self,
                                                        "Save parameter",
                                                        self.__current_path,
                                                        "YAML files (*.yaml);;All files (*)")
            if fileName:
                self.__current_path = os.path.dirname(fileName)
                nm.settings().current_dialog_path = os.path.dirname(fileName)
                content = self.content.value(with_tags=True)
                buf = ruamel.yaml.compat.StringIO()
                ruamel.yaml.dump(content, buf, Dumper=ruamel.yaml.RoundTripDumper)
                with open(fileName, 'w+') as f:
                    f.write(buf.getvalue())
        except Exception as e:
            print(traceback.format_exc(3))
            MessageBox.warning(self, "Save parameter Error",
                               'Error while save parameter',
                               utf8(e))

    def _load_parameter(self):
        try:
            (fileName, _) = QFileDialog.getOpenFileName(self, "Load parameter",
                                                              self.__current_path,
                                                              "YAML files (*.yaml);;All files (*)")
            if fileName:
                self.__current_path = os.path.dirname(fileName)
                nm.settings().current_dialog_path = os.path.dirname(fileName)
                with open(fileName, 'r') as f:
                    # print yaml.load(f.read())
                    self.content.set_values(ruamel.yaml.load(f.read(), Loader=ruamel.yaml.Loader))
        except Exception as e:
            print(traceback.format_exc())
            MessageBox.warning(self, "Load parameter Error",
                               'Error while load parameter',
                               utf8(e))


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%% close handling                        %%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    def _store_geometry(self):
        if self._geometry_name:
            settings = nm.settings().qsettings(nm.settings().CFG_GUI_FILE)
            settings.beginGroup(self._geometry_name)
            settings.setValue("size", self.size())
            settings.setValue("pos", self.pos())
            settings.endGroup()

    def accept(self):
        self._store_geometry()
        self.setResult(QDialog.Accepted)
        self.accepted.emit()
        if self.isModal():
            self.hide()

    def reject(self):
        self._store_geometry()
        self.setResult(QDialog.Rejected)
        self.rejected.emit()
        self.hide()

    def hideEvent(self, event):
        self.close()

    def closeEvent(self, event):
        '''
        Test the open files for changes and save this if needed.
        '''
        self.setAttribute(Qt.WA_DeleteOnClose, True)
        if self.result() == QDialog.Accepted:
            event.setAccepted(False)
        QDialog.closeEvent(self, event)


class MasterParameterDialog(ParameterDialog):
    '''
    This dialog is an extension to the L{ParameterDialog}. The parameter and their
    values are requested from the ROS master parameter server. The requests are
    threaded and allows the also threaded changed of ROS parameter assigned to
    given namespace.
    '''

    def __init__(self, masteruri, ns='/', parent=None, store_geometry=''):
        '''
        :param str masteruri: if the master uri is not None, the parameter are retrieved from ROS parameter server.
        :param str ns: namespace of the parameter retrieved from the ROS parameter server.
        '''
        ParameterDialog.__init__(self, dict(), parent=parent, store_geometry=store_geometry)
        self.masteruri = masteruri
        self.ns = ns
        self.is_delivered = False
        self.is_send = False
        self.mIcon = nm.settings().icon('default_cfg.png')
        self.setWindowIcon(self.mIcon)
        # self.resize(450, 300)
        self.add_new_button = QPushButton()
        self.add_new_button.setIcon(nm.settings().icon('crystal_clear_add.png'))
        self.add_new_button.clicked.connect(self._on_add_parameter)
        self.add_new_button.setToolTip('Adds a new parameter to the list')
        self.add_new_button.setFlat(True)
        self.buttonBox.addButton(self.add_new_button, QDialogButtonBox.ActionRole)
        self.showLoadSaveButtons()
#    self.apply_button = QPushButton(self.tr("&Ok"))
#    self.apply_button.clicked.connect(self._on_apply)
#    self.buttonBox.addButton(self.apply_button, QDialogButtonBox.ApplyRole)
#    self.buttonBox.accepted.connect(self._on_apply)
        self.setText(' '.join(['Obtaining parameters from the parameter server', masteruri, '...']))
        self.parameterHandler = ParameterHandler()
        self.parameterHandler.parameter_list_signal.connect(self._on_param_list)
        self.parameterHandler.parameter_values_signal.connect(self._on_param_values)
        self.parameterHandler.delivery_result_signal.connect(self._on_delivered_values)
        self.parameterHandler.requestParameterList(masteruri, ns)
#    self.apply_button.setFocus(Qt.OtherFocusReason)

    def accept(self):
        if self.masteruri is not None and not self.is_send:
            try:
                params = self.getKeywords(True)
                params = self.keywords2params(params)
                ros_params = dict()
                for p, v in params.items():
                    rospy.logdebug("updated parameter: %s, %s, %s", p, utf8(v), type(v))
                    ros_params[roslib.names.ns_join(self.ns, p)] = v
                if ros_params:
                    self.is_send = True
                    self.setText('Sends parameters to the server...')
                    self.parameterHandler.deliverParameter(self.masteruri, ros_params)
                else:
                    self.close()
            except Exception as e:
                print(traceback.format_exc(3))
                MessageBox.warning(self, self.tr("Warning"), utf8(e))
        elif self.masteruri is None:
            MessageBox.warning(self, self.tr("Error"), 'Invalid ROS master URI')

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%          ROS parameter handling       %%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def _on_add_parameter(self):
        params_arg = {'namespace': {':type': 'string', ':value': self.ns},
                      'name': {':type': 'string', ':value': ''},
                      'type': {':type': 'string', ':value': ['string', 'int', 'float', 'bool', 'list']},
                      'value': {':type': 'string', ':value': ''}
                      }
        dia = ParameterDialog(params_arg, store_geometry='add_parameter_in_master_dialog')
        dia.setWindowTitle('Add new parameter')
        dia.setFilterVisible(False)
        if dia.exec_():
            try:
                params = dia.getKeywords()
                if params['name']:
                    if params['type'] == 'int':
                        value = int(params['value'])
                    elif params['type'] == 'float':
                        value = float(params['value'])
                    elif params['type'] == 'bool':
                        value = str2bool(params['value'])
                    elif params['type'] == 'list':
                        try:
                            value = ruamel.yaml.load("[%s]" % params['value'], Loader=ruamel.yaml.Loader)
                            # if there is no YAML, load() will return an
                            # empty string.  We want an empty dictionary instead
                            # for our representation of empty.
                            if value is None:
                                value = []
                        except ruamel.yaml.MarkedYAMLError as e:
                            MessageBox.warning(self, self.tr("Warning"), "yaml error: %s" % utf8(e))
                    else:
                        value = params['value']
                    self._on_param_values(self.masteruri, 1, '', {roslib.names.ns_join(params['namespace'], params['name']): (1, '', value)}, new_param=True)
                else:
                    MessageBox.warning(self, self.tr("Warning"), 'Empty name is not valid!')
            except ValueError as e:
                print(traceback.format_exc(3))
                MessageBox.warning(self, self.tr("Warning"), utf8(e))

    def _on_param_list(self, masteruri, code, msg, params):
        '''
        :param str masteruri: The URI of the ROS parameter server
        :param int code: The return code of the request. If not 1, the message is set and the list can be ignored.
        :param str msg: The message of the result.
        :param [str] params: The list the parameter names.
        '''
        if code == 1:
            params.sort()
            self.parameterHandler.requestParameterValues(masteruri, params)
        else:
            self.setText(msg)

    def _on_param_values(self, masteruri, code, msg, params, new_param=False):
        '''
        :param str masteruri: The URI of the ROS parameter server
        :param int code: The return code of the request. If not 1, the message is set and the list can be ignored.
        :param str msg: The message of the result.
        :param params: The dictionary the parameter names and request result.
        :type params: dict(paramName : (code, statusMessage, parameterValue))
        '''
        if code == 1:
            dia_params = dict()
            for p, (code_n, _, val) in params.items():  # _:=msg_n
                if code_n != 1:
                    val = ''
                type_str = 'string'
                value = utf8(val)
                if isinstance(val, bool):
                    type_str = 'bool'
                elif isinstance(val, int):
                    type_str = 'int'
                elif isinstance(val, float):
                    type_str = 'float'
                elif isinstance(val, list) or isinstance(val, dict):
                    # handle representation of `rosparam`
                    type_str = 'list'
                    value = ''
                    for v in val:
                        if len(value) > 0:
                            value = value + ', '
                        value = value + utf8(v)
                elif isinstance(val, xmlrpcclient.Binary):
                    type_str = 'binary'
                param = p.replace(self.ns, '')
                param = param.strip(roslib.names.SEP)
                names_sep = param.split(roslib.names.SEP)
                param_name = names_sep.pop()
                if names_sep:
                    group = dia_params
                    for n in names_sep:
                        group_name = n
                        if group_name in group:
                            group = group[group_name]
                        else:
                            tmp_dict = dict()
                            group[group_name] = tmp_dict
                            group = tmp_dict
                    group[param_name] = {':type': type_str, ':value': value}
                else:
                    dia_params[param_name] = {':type': type_str, ':value': value}
            try:
                self.content.createFieldFromValue(dia_params, clear_origin_value=new_param)
                self.setInfoActive(False)
            except Exception as e:
                print(traceback.format_exc(3))
                MessageBox.warning(self, self.tr("Warning"), utf8(e))
        else:
            self.setText(msg)

    def _on_delivered_values(self, masteruri, code, msg, params):
        '''
        :param str masteruri: The URI of the ROS parameter server
        :param int code: The return code of the request. If not 1, the message is set and the list can be ignored.
        :param str msg: The message of the result.
        :param params: The dictionary the parameter names and request result.
        :type params: dict(paramName : (code, statusMessage, parameterValue))
        '''
        self.is_delivered = True
        errmsg = ''
        if code == 1:
            for _, (code_n, msg, _) in params.items():  # _:=param, val
                if code_n != 1:
                    errmsg = '\n'.join([errmsg, msg])
        else:
            errmsg = msg if msg else 'Unknown error on set parameter'
        if errmsg:
            print(traceback.format_exc(2))
            MessageBox.warning(self, self.tr("Warning"), utf8(errmsg))
            self.is_delivered = False
            self.is_send = False
            self.setInfoActive(False)
        if self.is_delivered:
            self.close()


class ServiceDialog(ParameterDialog):
    '''
    Adds a support for calling a service to the L{ParameterDialog}. The needed
    input fields are created from the service request message type. The service
    call is executed in a thread to avoid blocking GUI.
    '''
    service_resp_signal = Signal(str, str)

    def __init__(self, service, parent=None):
        '''
        :param service: Service to call.
        :type service: U{fkie_master_discovery.ServiceInfo<http://docs.ros.org/kinetic/api/fkie_master_discovery/html/modules.html#fkie_master_discovery.master_info.ServiceInfo>}
        '''
        self.service = service
        slots = service.get_service_class(True)._request_class.__slots__
        types = service.get_service_class()._request_class._slot_types
        ParameterDialog.__init__(self, self._params_from_slots(slots, types), buttons=QDialogButtonBox.Close, parent=parent, store_geometry='service_call_dialog')
        self.setWindowTitle('Call %s' % service.name)
        self.service_resp_signal.connect(self._handle_resp)
        # self.resize(450, 300)
        if not slots:
            self.setText(''.join(['Wait for response ...']))
            thread = threading.Thread(target=self._callService)
            thread.setDaemon(True)
            thread.start()
        else:
            self.call_service_button = QPushButton(self.tr("&Call"))
            self.call_service_button.clicked.connect(self._on_call_service)
            self.buttonBox.addButton(self.call_service_button, QDialogButtonBox.ActionRole)
            self.hide_button = QPushButton(self.tr("&Hide/Show output"))
            self.hide_button.clicked.connect(self._on_hide_output)
            self.buttonBox.addButton(self.hide_button, QDialogButtonBox.ActionRole)
            self.hide_button.setVisible(False)
            self.showLoadSaveButtons()

    def _on_hide_output(self):
        self.setInfoActive(not self.info_field.isVisible())

    def _on_call_service(self):
        try:
            self.hide_button.setVisible(True)
            params = self.getKeywords()
            self.setText(''.join(['Wait for response ...']))
            thread = threading.Thread(target=self._callService, args=((params,)))
            thread.setDaemon(True)
            thread.start()
        except Exception as e:
            rospy.logwarn("Error while reading parameter for %s service: %s", utf8(self.service.name), utf8(e))
            self.setText(''.join(['Error while reading parameter:\n', utf8(e)]))

    def _callService(self, params={}):
        req = utf8(params) if params else ''
        try:
            req, resp = nm.starter().callService(self.service.uri, self.service.name, self.service.get_service_class(), [params])
            self.service_resp_signal.emit(utf8(repr(req)), utf8(repr(resp)))
        except Exception as e:
            print(traceback.format_exc(2))
            rospy.logwarn("Error while call service '%s': %s", utf8(self.service.name), utf8(e))
            self.service_resp_signal.emit(utf8(repr(req)), utf8(e))

    @classmethod
    def _params_from_slots(cls, slots, types, values={}):
        result = dict()
        for slot, msg_type in zip(slots, types):
            base_type, is_array, _array_length = roslib.msgs.parse_type(msg_type)
            if base_type in roslib.msgs.PRIMITIVE_TYPES or base_type in ['time', 'duration']:
                default_value = 'now' if base_type in ['time', 'duration'] else ''
                if slot in values and values[slot]:
                    default_value = values[slot]
                result[slot] = {':type': msg_type, ':value': default_value}
            else:
                try:
                    list_msg_class = roslib.message.get_message_class(base_type)
                    if is_array and slot in values:
                        subresult = []
                        for slot_value in values[slot]:
                            subvalue = cls._params_from_slots(list_msg_class.__slots__, list_msg_class._slot_types, slot_value if slot in values and slot_value else {})
                            subresult.append(subvalue)
                        result[slot] = {':value': subresult, ':type': msg_type}
                    else:
                        subresult = cls._params_from_slots(list_msg_class.__slots__, list_msg_class._slot_types, values[slot] if slot in values and values[slot] else {})
                        if is_array:
                            result[slot] = {':value': subresult, ':type': msg_type}
                        else:
                            subresult[':type'] = msg_type
                            result[slot] = subresult
                except ValueError as e:
                    print(traceback.format_exc())
                    rospy.logwarn("Error while parse message type '%s': %s", utf8(msg_type), utf8(e))
        return result

    def _handle_resp(self, req, resp):
        self.setWindowTitle(''.join(['Request / Response of ', self.service.name]))
        # replace some of Escape Characters
        resp_str = utf8(resp).replace('\\r\\n', '\n')
        resp_str = resp_str.replace('\\n', '\n')
        resp_str = resp_str.replace('\\t', '\t')
        resp_str = resp_str.replace('\\v', '\v')
        self.setText('\n'.join([utf8(req), '---', resp_str]))
