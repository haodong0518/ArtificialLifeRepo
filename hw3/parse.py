import xml.dom.minidom

dom = xml.dom.minidom.parse("actuator.XML") # or xml.dom.minidom.parseString(xml_string)
pretty_xml_as_string = dom.toprettyxml()
print(pretty_xml_as_string)