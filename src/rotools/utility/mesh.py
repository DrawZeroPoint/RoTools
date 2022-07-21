from __future__ import print_function


class WebotsConverter(object):
    """This tool converts solid mesh in Webots proto file to stl format in
    a semi-automatic way. First, you need copy the vertexes and coordinate index
    of a mesh to x.vertex and x.index manually, and then using this tool to
    generate stl in ASCII format. You may need to further convert the stl file
    to binary using Blender to save space.
    """

    def __init__(self):
        super(WebotsConverter, self).__init__()
        self.face_list = []

    def read_proto(self, vertex_file, index_file):
        """Given a webots proto, you need manually copy the coord Coordinate ->
        point and coordIndex values into file vertex_file and index_file, respectively.
        Remove the last ', -1' in index_file.

        :param vertex_file: string file contains all vertexes coordinates
        :param index_file: string file records vertex index of each face
        :return:
        """
        point_list = self.read_coordinate_file(vertex_file)
        index_list = self.read_index_file(index_file)

        for i in index_list:
            # index of the vertexes belonging to the same face
            try:
                i1, i2, i3 = i.split(" ")
                v1 = point_list[int(i1)]
                v2 = point_list[int(i2)]
                v3 = point_list[int(i3)]
                self.face_list.append([v1, v2, v3])
            except ValueError:
                print(i)

    @staticmethod
    def read_coordinate_file(coordinate_file):
        """The coordinates of vertexes are arranged in x y z order.
        Note that ',' may be present after z.

        :param coordinate_file: text file containing the coordinate values
        :return:
        """
        with open(coordinate_file, "r") as v_file:
            coord_raw = v_file.readline()
            coordinates = coord_raw.replace(",", "")
            c_list = coordinates.split(" ")
            # 3 coordinates form a point
            assert len(c_list) % 3 == 0
            sz = len(c_list)
            point_list = []
            for i in range(0, sz, 3):
                point_list.append(
                    "{} {} {}".format(c_list[i], c_list[i + 1], c_list[i + 2])
                )
            print("Total vertexes #", len(point_list))
            return point_list

    @staticmethod
    def read_index_file(index_file):
        """The usable values in the index_file are arranged in the order
        v1, v2, v3, -1, note that any of these 4 ',' could be missing.

        :param index_file: text file containing the coordIndex
        :return: index_list
        """
        with open(index_file, "r") as i_file:
            indexes_raw = i_file.readline()
            indexes = indexes_raw.replace(",", "")
            index_list = indexes.split(" -1 ")
            return index_list

    def write_to_stl(self, stl_file):
        """

        facet normal -1.000000 0.000000 0.000000
            outer loop
            vertex -1.000000 -1.000000 -1.000000
            vertex -1.000000 -1.000000 1.000000
            vertex -1.000000 1.000000 1.000000
            endloop
        endfacet

        :param stl_file:
        :return:
        """
        with open(stl_file, "w") as s_file:
            s_file.write("solid Exported from RoTools\n")
            for face in self.face_list:
                # s_file.write('face normal {} {} {}'.format(nx, ny, nz))
                s_file.write("outer loop\n")
                s_file.write("vertex {}\n".format(face[0]))
                s_file.write("vertex {}\n".format(face[1]))
                s_file.write("vertex {}\n".format(face[2]))
                s_file.write("endloop\n")
                s_file.write("endfacet\n")
            s_file.write("endsolid Exported from RoTools")
