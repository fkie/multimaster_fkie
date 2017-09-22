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

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QBrush, QColor, QIcon, QPalette
from xmlrpclib import Binary
import os
import roslib.msgs
import roslib.names
import rospy
import sys
import threading

from node_manager_fkie.common import utf8
from node_manager_fkie.detailed_msg_box import MessageBox
from node_manager_fkie.editor.line_edit import EnchancedLineEdit
from node_manager_fkie.parameter_handler import ParameterHandler

import node_manager_fkie as nm
try:
    from python_qt_binding.QtGui import QApplication, QComboBox, QCheckBox, QLineEdit, QScrollArea, QWidget
    from python_qt_binding.QtGui import QFormLayout, QHBoxLayout, QVBoxLayout, QSpacerItem, QSizePolicy
    from python_qt_binding.QtGui import QFrame, QDialog, QDialogButtonBox, QFileDialog, QLabel, QPushButton, QTextEdit
except:
    from python_qt_binding.QtWidgets import QApplication, QComboBox, QCheckBox, QLineEdit, QScrollArea, QWidget
    from python_qt_binding.QtWidgets import QFormLayout, QHBoxLayout, QVBoxLayout, QSpacerItem, QSizePolicy
    from python_qt_binding.QtWidgets import QFrame, QDialog, QDialogButtonBox, QFileDialog, QLabel, QPushButton, QTextEdit


def str2bool(val):
    return val.lower() in ("yes", "true", "t", "1")


class MyComboBox(QComboBox):

    remove_item_signal = Signal(str)

    def __init__(self, parent=None):
        QComboBox.__init__(self, parent=parent)
        self.parameter_description = None

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
            except:
                import traceback
                print traceback.format_exc(1)
        QComboBox.keyPressEvent(self, event)


class ParameterDescription(object):
    '''
    Used for internal representation of the parameter in dialog.
    '''

    def __init__(self, name, msg_type, value=None, widget=None):
        self._name = str(name)
        self._type = msg_type
        if isinstance(self._type, dict):
            self._type = 'dict'
        elif isinstance(self._type, list):
            self._type = 'list'
        self._value = value
        self._value_org = value
        self._widget = widget
        try:
            self._base_type, self._is_array_type, self._array_length = roslib.msgs.parse_type(self._type)
        except:
            pass
        if msg_type == 'binary':
            self._base_type = msg_type

    def __repr__(self):
        return ''.join([self._name, ' [', self._type, ']'])

    def origin_value(self):
        return self._value_org

    def changed(self):
        return utf8(self.origin_value()) != utf8(self._value)

    def name(self):
        return self._name

    def setWidget(self, widget):
        self._widget = widget
        if widget is not None:
            widget.parameter_description = self
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
        return self._is_array_type or self._type in ['[]']

    def arrayLength(self):
        return self._array_length

    def isPrimitiveType(self):
        result = self._base_type in roslib.msgs.PRIMITIVE_TYPES
        result = result or self._base_type in ['int', 'float', 'time', 'duration', 'binary']
        # if value is a string, the list is represented as a string, see `rosparam`
        result = result or self._type in ['[]']
        return result

    def isTimeType(self):
        return self._base_type in ['time', 'duration']

    def isBinaryType(self):
        return self._base_type in ['binary']

    def baseType(self):
        return self._base_type

    def updateValueFromField(self):
        field = self.widget()
        result = ''
        if isinstance(field, QCheckBox):
            result = repr(field.isChecked())
        elif isinstance(field, QLineEdit):
            result = field.text()
        elif isinstance(field, QComboBox):
            result = field.currentText()
        self.updateValue(result)

    def updateValue(self, value):
        try:
            if isinstance(value, (dict, list)):
                self._value = value
            elif value:
                nm.history().addParamCache(self.fullName(), value)
                if self.isArrayType():
                    if 'int' in self.baseType() or 'byte' in self.baseType():
                        self._value = map(int, value.lstrip('[').rstrip(']').split(','))
                    elif 'float' in self.baseType():
                        self._value = map(float, value.lstrip('[').rstrip(']').split(','))
                    elif 'bool' in self.baseType():
                        self._value = map(str2bool, value.lstrip('[').rstrip(']').split(','))
                    elif self.isBinaryType():
                        self._value = value
                    else:
                        try:
                            import yaml
                            self._value = yaml.load("[%s]" % value)
                            # if there is no YAML, load() will return an
                            # empty string.  We want an empty dictionary instead
                            # for our representation of empty.
                            if self._value is None:
                                self._value = []
                        except yaml.MarkedYAMLError, e:
                            raise Exception("Field [%s] yaml error: %s" % (self.fullName(), utf8(e)))
                    if not self.arrayLength() is None and self.arrayLength() != len(self._value):
                        raise Exception(''.join(["Field [", self.fullName(), "] has incorrect number of elements: ", utf8(len(self._value)), " != ", str(self.arrayLength())]))
                else:
                    if 'int' in self.baseType() or 'byte' in self.baseType():
                        self._value = int(value)
                    elif 'float' in self.baseType():
                        self._value = float(value)
                    elif 'bool' in self.baseType():
                        if isinstance(value, bool):
                            self._value = value
                        else:
                            self._value = str2bool(value)
                    elif self.isBinaryType():
                        self._value = utf8(value)
                    elif self.isTimeType():
                        if value == 'now':
                            self._value = 'now'
                        else:
                            try:
                                val = eval(value)
                                if isinstance(val, dict):
                                    self._value = val
                                else:
                                    secs = int(val)
                                    nsecs = int((val - secs) * 1000000000)
                                    self._value = {'secs': secs, 'nsecs': nsecs}
                            except Exception:
                                self._value = {'secs': 0, 'nsecs': 0}
                    else:
                        self._value = value.encode(sys.getfilesystemencoding())
            else:
                if self.isArrayType():
                    arr = []
                    self._value = arr
                else:
                    if 'int' in self.baseType() or 'byte' in self.baseType():
                        self._value = 0
                    elif 'float' in self.baseType():
                        self._value = 0.0
                    elif 'bool' in self.baseType():
                        self._value = False
                    elif self.isBinaryType():
                        self._value = utf8(value)
                    elif self.isTimeType():
                        self._value = {'secs': 0, 'nsecs': 0}
                    else:
                        self._value = ''
            nm.history().addParamCache(self.fullName(), value)
        except Exception, e:
            raise Exception("Error while set value '%s', for '%s': %s" % (utf8(value), self.fullName(), utf8(e)))
        return self._value

    def value(self):
        if not self.isPrimitiveType() and not self.widget() is None:
            return self.widget().value()
        elif self.isPrimitiveType():
            self.updateValueFromField()
#      if self.isTimeType() and self._value == 'now':
#        # FIX: rostopic does not support 'now' values in sub-headers
#        t = time.time()
#        return ({'secs': int(t), 'nsecs': int((t-int(t))*1000000)}, self.changed())
        return (self._value, self.changed())

    def removeCachedValue(self, value):
        nm.history().removeParamCache(self.fullName(), value)

    def createTypedWidget(self, parent):
        result = None
        if self.isPrimitiveType():
            value = self._value
            if 'bool' in self.baseType():
                result = QCheckBox(parent=parent)
                result.setObjectName(self.name())
                if not isinstance(value, bool):
                    value = str2bool(value[0] if isinstance(value, list) else value)
                self._value_org = value
                result.setChecked(value)
            else:
                result = MyComboBox(parent=parent)
                result.setObjectName(self.name())
                result.setSizePolicy(QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed))
                result.setEditable(True)
                result.remove_item_signal.connect(self.removeCachedValue)
                items = []
                if isinstance(value, list):
                    if self.isArrayType():
                        items.append(','.join([utf8(val) for val in value]))
                    else:
                        items[len(items):] = value
                else:
                    if value is not None and value:
                        items.append(utf8(value) if not isinstance(value, Binary) else '{binary data!!! updates will be ignored!!!}')
                    elif self.isTimeType():
                        items.append('now')
                self._value_org = items[0] if items else ''
                result.addItems(items)
        else:
            if self.isArrayType():
                result = ArrayBox(self.name(), self._type, parent=parent)
            else:
                result = GroupBox(self.name(), self._type, parent=parent)
        return result

    def addCachedValuesToWidget(self):
        if isinstance(self.widget(), QComboBox):
            values = nm.history().cachedParamValues(self.fullName())
            for i in range(self.widget().count()):
                try:
                    values.remove(self.widget().itemText(i))
                except:
                    pass
            if self.widget().count() == 0:
                values.insert(0, '')
            self.widget().addItems(values)


class MainBox(QWidget):
    '''
    Groups the parameter without visualization of the group. It is the main widget.
    '''

    def __init__(self, name, param_type, collapsible=True, parent=None):
        QWidget.__init__(self, parent)
        self.setObjectName(name)
        self.name = name
        self.type = param_type
        self.params = []
        self.collapsed = False
        self.parameter_description = None
        vLayout = QVBoxLayout()
        vLayout.setSpacing(0)
        self.options_layout = QHBoxLayout()
        self.param_widget = QFrame()
        self.name_label = QLabel(name)
        font = self.name_label.font()
        font.setBold(True)
        self.name_label.setFont(font)
        self.type_label = QLabel(''.join([' (', param_type, ')']))

        if collapsible:
            self.hide_button = QPushButton('-')
            self.hide_button.setFlat(True)
            self.hide_button.setMaximumSize(20, 20)
            self.hide_button.clicked.connect(self._on_hide_clicked)
            self.options_layout.addWidget(self.hide_button)
            self.options_layout.addWidget(self.name_label)
            self.options_layout.addWidget(self.type_label)
            self.options_layout.addStretch()

            vLayout.addLayout(self.options_layout)

            self.param_widget.setFrameShape(QFrame.Box)
            self.param_widget.setFrameShadow(QFrame.Raised)

        boxLayout = QFormLayout()
        boxLayout.setVerticalSpacing(0)
        self.param_widget.setLayout(boxLayout)
        vLayout.addWidget(self.param_widget)
        self.setLayout(vLayout)
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

    def createFieldFromValue(self, value):
        self.setUpdatesEnabled(False)
        try:
            if isinstance(value, (dict, list)):
                self._createFieldFromDict(value)
        finally:
            self.setUpdatesEnabled(True)

    def _createFieldFromDict(self, value, layout=None):
        if layout is None:
            layout = self.param_widget.layout()
        # sort the items: 1. header, 2. all premitives (sorted), 3. list, dict (sorted)
        all_params = []
        primitives = []
        komplex = []
        for name, (_type, val) in value.items():
            if _type in ['std_msgs/Header']:
                all_params.append((name, _type, val))
            elif isinstance(val, (dict, list)):
                komplex.append((name, _type, val))
            else:
                primitives.append((name, _type, val))
        all_params.extend(sorted(primitives))
        all_params.extend(sorted(komplex))

        # create widgets
        for name, _type, val in all_params:
            field = self.getField(name)
            if field is None:
                param_desc = ParameterDescription(name, _type, val)
                field = param_desc.createTypedWidget(self)
                param_desc.setWidget(field)
                self.params.append(param_desc)
                if isinstance(field, (GroupBox, ArrayBox)):
                    field.createFieldFromValue(val)
                    layout.addRow(field)
                else:
                    label_name = name if _type == 'string' else ''.join([name, ' (', _type, ')'])
                    label = QLabel(label_name, self)
                    label.setObjectName(''.join([name, '_label']))
                    label.setBuddy(field)
                    layout.addRow(label, field)
            else:
                if isinstance(field, (GroupBox, ArrayBox)):
                    field.createFieldFromValue(val)
                else:
                    raise Exception(''.join(["Parameter with name '", name, "' already exists!"]))

    def value(self):
        result = dict()
        for param in self.params:
            if not param.isBinaryType():
                result[param.name()] = param.value()
        return result

    def set_values(self, values):
        '''
        Sets the values for existing fields.
        :param values: the dictionary with values to set.
        :type values: dict
        :raise Exception: on errors
        '''
        if isinstance(values, dict):
            for param, val in values.items():
                value = val
                _type = 'unknown'
                if isinstance(val, tuple):
                    (_type, value) = val
                field = self.getField(param)
                if field is not None:
                    if isinstance(field, (GroupBox, ArrayBox)):
                        field.set_values(value)
                    else:
                        if isinstance(field, QCheckBox):
                            if not isinstance(value, bool):
                                value = str2bool(value[0] if isinstance(value, list) else value)
                            field.setChecked(value)
                        elif isinstance(field, QLineEdit):
                            # avoid ' or " that escapes the string values
                            field.setText(', '.join([utf8(v) for v in value]) if isinstance(value, list) else utf8(value))
                        elif isinstance(field, QComboBox):
                            field.setEditText(', '.join([utf8(v) for v in value]) if isinstance(value, list) else utf8(value))
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

    def filter(self, arg):
        '''
        Hide the parameter input field, which label dosn't contains the C{arg}.
        @param arg: the filter text
        @type arg: C{str}
        '''
        result = False
        for child in self.param_widget.children():
            if isinstance(child, (MainBox, GroupBox, ArrayBox)):
                show = not arg or child.objectName().lower().find(arg.lower()) != -1
                show = child.filter(arg) or show
                # hide group, if no parameter are visible
                child.setVisible(show)
                if show:
                    child.setCollapsed(False)
                    result = True
            elif isinstance(child, (QWidget)) and not isinstance(child, (QLabel)) and not isinstance(child, (QFrame)):
                label = child.parentWidget().layout().labelForField(child)
                if label is not None:
                    has_text = child.objectName().lower().find(arg.lower()) == -1
                    show = not arg or (not has_text or (hasattr(child, 'currentText') and not has_text))
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
        MainBox.__init__(self, ''.join(['#', utf8(index)]), param_type, True, parent)
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

    def value(self):
        result = dict()
        for param in self.params:
            result[param.name()] = param.value()
        return result


class ArrayBox(MainBox):
    '''
    Groups the parameter of a list.
    '''

    def __init__(self, name, param_type, parent=None):
        MainBox.__init__(self, name, param_type, True, parent)
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

    def _on_add_dynamic_entry(self):
        self.setUpdatesEnabled(False)
        try:
            if self._dynamic_value is not None:
                for v in self._dynamic_value:
                    if isinstance(v, dict):
                        entry_frame = ArrayEntry(self._dynamic_items_count, self.type)
                        self.param_widget.layout().addRow(entry_frame)
                        entry_frame._createFieldFromDict(v)
                        self._dynamic_items_count += 1
                        self.count_label.setText(utf8(self._dynamic_items_count))
                        break
        finally:
            self.setUpdatesEnabled(True)

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
            except:
                import traceback
                print traceback.format_exc(1)
            self.count_label.setText(utf8(self._dynamic_items_count))

    def createFieldFromValue(self, value):
        self.setUpdatesEnabled(False)
        try:
            if isinstance(value, list):
                self.addDynamicBox()
                self._dynamic_value = value
                self.set_values(value)
        finally:
            self.setUpdatesEnabled(True)

    def value(self):
        '''
        Goes through the list and creates dictionary with values of each element.
        '''
        result = list()
        for i in range(self.param_widget.layout().rowCount()):
            item = self.param_widget.layout().itemAt(i, QFormLayout.SpanningRole)
            if item and isinstance(item.widget(), ArrayEntry):
                result.append(item.widget().value())
        return result

    def set_values(self, values):
        '''
        Create a list of the elements and sets their values.
        :param values: The list of dictionaries with parameter values
        :type values: list
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
                    self._on_add_dynamic_entry()
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

    def __init__(self, params=dict(), buttons=QDialogButtonBox.Cancel | QDialogButtonBox.Ok, sidebar_var='', parent=None):
        '''
        Creates an input dialog.
        @param params: a dictionary with parameter names and (type, values).
        The C{value}, can be a primitive value, a list with values or parameter
        dictionary to create groups. In this case the type is the name of the group.
        @type params: C{dict(str:(str, {value, [..], dict()}))}
        '''
        QDialog.__init__(self, parent=parent)
        self.setObjectName('ParameterDialog - %s' % utf8(params))

        self.__current_path = nm.settings().current_dialog_path
        self.horizontalLayout = QHBoxLayout(self)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.horizontalLayout.setContentsMargins(1, 1, 1, 1)
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.verticalLayout.setContentsMargins(1, 1, 1, 1)
        # add filter row
        self.filter_frame = QFrame(self)
        filterLayout = QHBoxLayout(self.filter_frame)
        filterLayout.setContentsMargins(1, 1, 1, 1)
        label = QLabel("Filter:", self.filter_frame)
        self.filter_field = EnchancedLineEdit(self.filter_frame)
        filterLayout.addWidget(label)
        filterLayout.addWidget(self.filter_field)
        self.filter_field.textChanged.connect(self._on_filter_changed)
        self.filter_visible = True

        self.verticalLayout.addWidget(self.filter_frame)

        # create area for the parameter
        self.scrollArea = scrollArea = ScrollArea(self)
        scrollArea.setObjectName("scrollArea")
        scrollArea.setWidgetResizable(True)
        self.content = MainBox('/', 'str', False, self)
        scrollArea.setWidget(self.content)
        self.verticalLayout.addWidget(scrollArea)

        # add info text field
        self.info_field = QTextEdit(self)
        self.info_field.setVisible(False)
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
        self.sidebar_frame = QFrame()
        self.sidebar_frame.setObjectName(sidebar_var)
        sidebarframe_verticalLayout = QVBoxLayout(self.sidebar_frame)
        sidebarframe_verticalLayout.setObjectName("sidebarframe_verticalLayout")
        sidebarframe_verticalLayout.setContentsMargins(1, 1, 1, 1)
        self._sidebar_selected = 0
        if len(values) > 1 and sidebar_var in params:
            self.horizontalLayout.addWidget(self.sidebar_frame)
            try:
                self.sidebar_default_val = params[sidebar_var][1]
            except:
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
            self.content.createFieldFromValue(params)
            self.setInfoActive(False)

        if self.filter_frame.isVisible():
            self.filter_field.setFocus()
        self.setMinimumSize(350, 200)

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
            except:
                pass

    def showLoadSaveButtons(self):
        self.load_button = QPushButton()
        self.load_button.setIcon(QIcon(':/icons/load.png'))
        self.load_button.clicked.connect(self._load_parameter)
        self.load_button.setToolTip('Load parameters from YAML file')
        self.load_button.setFlat(True)
        self.buttonBox.addButton(self.load_button, QDialogButtonBox.ActionRole)
        self.save_button = QPushButton()
        self.save_button.clicked.connect(self._save_parameter)
        self.save_button.setIcon(QIcon(':/icons/save.png'))
        self.save_button.setToolTip('Save parameters to YAML file')
        self.save_button.setFlat(True)
        self.buttonBox.addButton(self.save_button, QDialogButtonBox.ActionRole)

    def _on_filter_changed(self):
        self.content.filter(self.filter_field.text())

    def setFilterVisible(self, val):
        '''
        Shows or hides the filter row.
        '''
        self.filter_visible = val
        self.filter_frame.setVisible(val & self.scrollArea.isHidden())

    def add_warning(self, message):
        label = QLabel()
        label.setWordWrap(True)
        label.setText(''.join(["<font color='red'>Warning!\n", message, "</font>"]))
        self.verticalLayout.insertWidget(1, label)

    def setText(self, text):
        '''
        Adds a label to the dialog's layout and shows the given text.
        @param text: the text to add to the dialog
        @type text: C{str}
        '''
        self.info_field.setText(text)
        self.setInfoActive(True)

    def setInfoActive(self, val):
        '''
        Activates or deactivates the info field of this dialog. If info field is
        activated, the filter frame and the input field are deactivated.
        @type val: C{bool}
        '''
        if val and self.info_field.isHidden():
            self.filter_frame.setVisible(False & self.filter_visible)
            self.scrollArea.setVisible(False)
            self.info_field.setVisible(True)
        elif not val and self.scrollArea.isHidden():
            self.filter_frame.setVisible(True & self.filter_visible)
            self.scrollArea.setVisible(True)
            self.info_field.setVisible(False)
            if self.filter_frame.isVisible():
                self.filter_field.setFocus()

    def setFocusField(self, field_label):
        field = self.content.getField(field_label, recursive=True)
        if field is not None:
            field.setFocus()

    def getKeywords(self, only_changed=False):
        '''
        @param only_changed: requests only changed parameter
        @type only_changed: bool (Default: False)
        @returns: a directory with parameter and value for all entered fields.
        @rtype: C{dict(str(param) : str(value))}
        '''
        # get the results of sidebar
        sidebar_list = []
        sidebar_name = self.sidebar_frame.objectName()
        for j in range(self.sidebar_frame.layout().count() - 1):
            w = self.sidebar_frame.layout().itemAt(j).widget()
            if isinstance(w, QCheckBox):
                if w.checkState() == Qt.Checked:
                    sidebar_list.append((w.objectName(), True))
        result_value = self.content.value()
        # add the sidebar results
        if sidebar_name in result_value:
            # skip the default value, if elements are selected in the side_bar
            if len(sidebar_list) == 0 or self.sidebar_default_val != result_value[sidebar_name][0]:
                sidebar_list.append(result_value[sidebar_name])
            result_value[sidebar_name] = ([v for v, _ in set(sidebar_list)], True)  # _:=changed
        result = self._remove_change_state(result_value, only_changed)
        return result

    def keywords2params(self, keywords):
        '''
        Resolves the dictionary values to ROS parameter names.
        @param keywords: the result of the getKeywords
        @return: dictionary of (ROS parameter name : value)
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

    def _remove_change_state(self, params, only_changed):
        result = dict()
        for param, value in params.items():
            if isinstance(value, dict):
                r = self._remove_change_state(value, only_changed)
                if r:
                    result[param] = r
            elif isinstance(value, list):
                new_val = []
                for val in value:
                    r = self._remove_change_state(val, only_changed)
                    if r:
                        new_val.append(r)
                if new_val:
                    result[param] = new_val
            elif isinstance(value, tuple):
                if value[1] or not only_changed:
                    result[param] = value[0]
            else:
                print "unknown parameter: should not happens", param, value
        return result

    def _save_parameter(self):
        try:
            import yaml
            (fileName, _) = QFileDialog.getSaveFileName(self,
                                                        "Save parameter",
                                                        self.__current_path,
                                                        "YAML files (*.yaml);;All files (*)")
            if fileName:
                self.__current_path = os.path.dirname(fileName)
                nm.settings().current_dialog_path = os.path.dirname(fileName)
                content = self._remove_change_state(self.content.value(), False)
                text = yaml.dump(content, default_flow_style=False)
                with open(fileName, 'w+') as f:
                    f.write(text)
        except Exception as e:
            import traceback
            print traceback.format_exc(1)
            MessageBox.warning(self, "Save parameter Error",
                               'Error while save parameter',
                               utf8(e))

    def _load_parameter(self):
        try:
            import yaml
            (fileName, _) = QFileDialog.getOpenFileName(self, "Load parameter",
                                                              self.__current_path,
                                                              "YAML files (*.yaml);;All files (*)")
            if fileName:
                self.__current_path = os.path.dirname(fileName)
                nm.settings().current_dialog_path = os.path.dirname(fileName)
                with open(fileName, 'r') as f:
                    # print yaml.load(f.read())
                    self.content.set_values(yaml.load(f.read()))
        except Exception as e:
            import traceback
            print traceback.format_exc(1)
            MessageBox.warning(self, "Load parameter Error",
                               'Error while load parameter',
                               utf8(e))


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%% close handling                        %%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def accept(self):
        self.setResult(QDialog.Accepted)
        self.accepted.emit()
        if self.isModal():
            self.hide()

    def reject(self):
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
        QDialog.closeEvent(self, event)


class MasterParameterDialog(ParameterDialog):
    '''
    This dialog is an extension to the L{ParameterDialog}. The parameter and their
    values are requested from the ROS master parameter server. The requests are
    threaded and allows the also threaded changed of ROS parameter assigned to
    given namespace.
    '''

    def __init__(self, masteruri, ns='/', parent=None):
        '''
        @param masteruri: if the master uri is not None, the parameter are retrieved from ROS parameter server.
        @type masteruri: C{str}
        @param ns: namespace of the parameter retrieved from the ROS parameter server.
        @type ns: C{str}
        '''
        ParameterDialog.__init__(self, dict(), parent=parent)
        self.masteruri = masteruri
        self.ns = ns
        self.is_delivered = False
        self.is_send = False
        self.mIcon = QIcon(":/icons/default_cfg.png")
        self.setWindowIcon(self.mIcon)
        self.resize(450, 300)
        self.add_new_button = QPushButton()
        self.add_new_button.setIcon(QIcon(':/icons/crystal_clear_add.png'))
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
            except Exception, e:
                import traceback
                print traceback.format_exc(1)
                MessageBox.warning(self, self.tr("Warning"), utf8(e))
        elif self.masteruri is None:
            MessageBox.warning(self, self.tr("Error"), 'Invalid ROS master URI')

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%          ROS parameter handling       %%%%%%%%%%%%%%%%%%%%%
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    def _on_add_parameter(self):
        params_arg = {'namespace': ('string', self.ns), 'name': ('string', ''), 'type': ('string', ['string', 'int', 'float', 'bool', 'list']), 'value': ('string', '')}
        dia = ParameterDialog(params_arg)
        dia.setWindowTitle('Add new parameter')
        dia.resize(360, 150)
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
                            import yaml
                            value = yaml.load("[%s]" % params['value'])
                            # if there is no YAML, load() will return an
                            # empty string.  We want an empty dictionary instead
                            # for our representation of empty.
                            if value is None:
                                value = []
                        except yaml.MarkedYAMLError, e:
                            MessageBox.warning(self, self.tr("Warning"), "yaml error: %s" % utf8(e))
                    else:
                        value = params['value']
                    self._on_param_values(self.masteruri, 1, '', {roslib.names.ns_join(params['namespace'], params['name']): (1, '', value)})
                else:
                    MessageBox.warning(self, self.tr("Warning"), 'Empty name is not valid!')
            except ValueError, e:
                import traceback
                print traceback.format_exc(1)
                MessageBox.warning(self, self.tr("Warning"), utf8(e))

    def _on_param_list(self, masteruri, code, msg, params):
        '''
        @param masteruri: The URI of the ROS parameter server
        @type masteruri: C{str}
        @param code: The return code of the request. If not 1, the message is set and the list can be ignored.
        @type code: C{int}
        @param msg: The message of the result.
        @type msg: C{str}
        @param params: The list the parameter names.
        @type params: C{[str]}
        '''
        if code == 1:
            params.sort()
            self.parameterHandler.requestParameterValues(masteruri, params)
        else:
            self.setText(msg)

    def _on_param_values(self, masteruri, code, msg, params):
        '''
        @param masteruri: The URI of the ROS parameter server
        @type masteruri: C{str}
        @param code: The return code of the request. If not 1, the message is set and the list can be ignored.
        @type code: C{int}
        @param msg: The message of the result.
        @type msg: C{str}
        @param params: The dictionary the parameter names and request result.
        @type params: C{dict(paramName : (code, statusMessage, parameterValue))}
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
                    type_str = '[]'
                    value = ''
                    for v in val:
                        if len(value) > 0:
                            value = value + ', '
                        value = value + utf8(v)
                elif isinstance(val, Binary):
                    type_str = 'binary'
                param = p.replace(self.ns, '')
                names_sep = param.split(roslib.names.SEP)
                param_name = names_sep.pop()
                if names_sep:
                    group = dia_params
                    for n in names_sep:
                        group_name = n
                        if group_name in group:
                            group = group[group_name][1]
                        else:
                            tmp_dict = dict()
                            group[group_name] = ('list', tmp_dict)
                            group = tmp_dict
                    group[param_name] = (type_str, [value])
                else:
                    dia_params[param_name] = (type_str, [value])
            try:
                self.content.createFieldFromValue(dia_params)
                self.setInfoActive(False)
            except Exception, e:
                import traceback
                print traceback.format_exc(1)
                MessageBox.warning(self, self.tr("Warning"), utf8(e))
        else:
            self.setText(msg)

    def _on_delivered_values(self, masteruri, code, msg, params):
        '''
        @param masteruri: The URI of the ROS parameter server
        @type masteruri: C{str}
        @param code: The return code of the request. If not 1, the message is set and the list can be ignored.
        @type code: C{int}
        @param msg: The message of the result.
        @type msg: C{str}
        @param params: The dictionary the parameter names and request result.
        @type params: C{dict(paramName : (code, statusMessage, parameterValue))}
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
            import traceback
            print traceback.format_exc(1)
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
        @param service: Service to call.
        @type service: U{master_discovery_fkie.ServiceInfo<http://docs.ros.org/kinetic/api/master_discovery_fkie/html/modules.html#master_discovery_fkie.master_info.ServiceInfo>}
        '''
        self.service = service
        slots = service.get_service_class(True)._request_class.__slots__
        types = service.get_service_class()._request_class._slot_types
        ParameterDialog.__init__(self, self._params_from_slots(slots, types), buttons=QDialogButtonBox.Close, parent=parent)
        self.setWindowTitle(''.join(['Call ', service.name]))
        self.service_resp_signal.connect(self._handle_resp)
        self.resize(450, 300)
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
        except Exception, e:
            rospy.logwarn("Error while reading parameter for %s service: %s", utf8(self.service.name), utf8(e))
            self.setText(''.join(['Error while reading parameter:\n', utf8(e)]))

    def _callService(self, params={}):
        req = utf8(params) if params else ''
        try:
            req, resp = nm.starter().callService(self.service.uri, self.service.name, self.service.get_service_class(), [params])
            self.service_resp_signal.emit(utf8(repr(req)), utf8(repr(resp)))
        except Exception, e:
            import traceback
            print traceback.format_exc(1)
            rospy.logwarn("Error while call service '%s': %s", utf8(self.service.name), utf8(e))
            self.service_resp_signal.emit(utf8(repr(req)), utf8(e))

    @classmethod
    def _params_from_slots(cls, slots, types, values={}):
        result = dict()
        for slot, msg_type in zip(slots, types):
            base_type, is_array, _ = roslib.msgs.parse_type(msg_type)  # _:=array_length
            if base_type in roslib.msgs.PRIMITIVE_TYPES or base_type in ['time', 'duration']:
                default_value = 'now' if base_type in ['time', 'duration'] else ''
                if slot in values and values[slot]:
                    default_value = values[slot]
                result[slot] = (msg_type, default_value)
            else:
                try:
                    list_msg_class = roslib.message.get_message_class(base_type)
                    if is_array and slot in values:
                        subresult = []
                        for slot_value in values[slot]:
                            subvalue = cls._params_from_slots(list_msg_class.__slots__, list_msg_class._slot_types, slot_value if slot in values and slot_value else {})
                            subresult.append(subvalue)
                        result[slot] = (msg_type, subresult)
                    else:
                        subresult = cls._params_from_slots(list_msg_class.__slots__, list_msg_class._slot_types, values[slot] if slot in values and values[slot] else {})
                        result[slot] = (msg_type, [subresult] if is_array else subresult)
                except ValueError, e:
                    import traceback
                    print traceback.format_exc(1)
                    rospy.logwarn("Error while parse message type '%s': %s", utf8(msg_type), utf8(e))
        return result

    def _handle_resp(self, req, resp):
        self.setWindowTitle(''.join(['Request / Response of ', self.service.name]))
        self.setText('\n'.join([utf8(req), '---', utf8(resp)]))
