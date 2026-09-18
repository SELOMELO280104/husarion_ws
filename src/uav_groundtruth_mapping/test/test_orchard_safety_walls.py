from pathlib import Path
import xml.etree.ElementTree as ET


def test_orchard_safety_walls_have_four_collisions_and_visuals():
    sdf_path = (
        Path(__file__).parents[1]
        / 'sdf'
        / 'orchard_safety_walls.sdf'
    )
    root = ET.parse(sdf_path).getroot()
    model = root.find('model')

    assert model is not None
    assert model.attrib['name'] == 'orchard_safety_walls'
    assert model.findtext('static') == 'true'

    link = model.find('link')
    collisions = link.findall('collision')
    visuals = link.findall('visual')
    expected = {'north_wall', 'south_wall', 'east_wall', 'west_wall'}

    assert {
        item.attrib['name'].removesuffix('_collision')
        for item in collisions
    } == expected
    assert {
        item.attrib['name'].removesuffix('_visual')
        for item in visuals
    } == expected

    for collision in collisions:
        size = [
            float(value)
            for value in collision.findtext('geometry/box/size').split()
        ]
        assert max(size[:2]) > 37.0
        assert size[2] == 1.5
