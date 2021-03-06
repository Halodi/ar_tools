# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
import grpc

import ar_tools.Common_pb2 as Common__pb2
import ar_tools.Geometry_pb2 as Geometry__pb2
import ar_tools.Image_pb2 as Image__pb2


class ImageServerStub(object):
    """Missing associated documentation comment in .proto file"""

    def __init__(self, channel):
        """Constructor.

        Args:
            channel: A grpc.Channel.
        """
        self.get_metadata = channel.unary_unary(
                '/halodi.protobuf.sensors.ImageServer/get_metadata',
                request_serializer=Common__pb2.Empty.SerializeToString,
                response_deserializer=Image__pb2.ImageMetadata.FromString,
                )
        self.get_metadatas = channel.unary_unary(
                '/halodi.protobuf.sensors.ImageServer/get_metadatas',
                request_serializer=Common__pb2.Empty.SerializeToString,
                response_deserializer=Image__pb2.ImagesMetadata.FromString,
                )
        self.get_sensor_pose = channel.unary_unary(
                '/halodi.protobuf.sensors.ImageServer/get_sensor_pose',
                request_serializer=Common__pb2.Empty.SerializeToString,
                response_deserializer=Geometry__pb2.PoseWithMetadata.FromString,
                )
        self.get_latest_timestamp = channel.unary_unary(
                '/halodi.protobuf.sensors.ImageServer/get_latest_timestamp',
                request_serializer=Common__pb2.Empty.SerializeToString,
                response_deserializer=Common__pb2.Timestamp.FromString,
                )
        self.stream = channel.unary_stream(
                '/halodi.protobuf.sensors.ImageServer/stream',
                request_serializer=Common__pb2.Empty.SerializeToString,
                response_deserializer=Common__pb2.DataChunk.FromString,
                )


class ImageServerServicer(object):
    """Missing associated documentation comment in .proto file"""

    def get_metadata(self, request, context):
        """Missing associated documentation comment in .proto file"""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def get_metadatas(self, request, context):
        """Missing associated documentation comment in .proto file"""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def get_sensor_pose(self, request, context):
        """Missing associated documentation comment in .proto file"""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def get_latest_timestamp(self, request, context):
        """Missing associated documentation comment in .proto file"""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')

    def stream(self, request, context):
        """Missing associated documentation comment in .proto file"""
        context.set_code(grpc.StatusCode.UNIMPLEMENTED)
        context.set_details('Method not implemented!')
        raise NotImplementedError('Method not implemented!')


def add_ImageServerServicer_to_server(servicer, server):
    rpc_method_handlers = {
            'get_metadata': grpc.unary_unary_rpc_method_handler(
                    servicer.get_metadata,
                    request_deserializer=Common__pb2.Empty.FromString,
                    response_serializer=Image__pb2.ImageMetadata.SerializeToString,
            ),
            'get_metadatas': grpc.unary_unary_rpc_method_handler(
                    servicer.get_metadatas,
                    request_deserializer=Common__pb2.Empty.FromString,
                    response_serializer=Image__pb2.ImagesMetadata.SerializeToString,
            ),
            'get_sensor_pose': grpc.unary_unary_rpc_method_handler(
                    servicer.get_sensor_pose,
                    request_deserializer=Common__pb2.Empty.FromString,
                    response_serializer=Geometry__pb2.PoseWithMetadata.SerializeToString,
            ),
            'get_latest_timestamp': grpc.unary_unary_rpc_method_handler(
                    servicer.get_latest_timestamp,
                    request_deserializer=Common__pb2.Empty.FromString,
                    response_serializer=Common__pb2.Timestamp.SerializeToString,
            ),
            'stream': grpc.unary_stream_rpc_method_handler(
                    servicer.stream,
                    request_deserializer=Common__pb2.Empty.FromString,
                    response_serializer=Common__pb2.DataChunk.SerializeToString,
            ),
    }
    generic_handler = grpc.method_handlers_generic_handler(
            'halodi.protobuf.sensors.ImageServer', rpc_method_handlers)
    server.add_generic_rpc_handlers((generic_handler,))


 # This class is part of an EXPERIMENTAL API.
class ImageServer(object):
    """Missing associated documentation comment in .proto file"""

    @staticmethod
    def get_metadata(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/halodi.protobuf.sensors.ImageServer/get_metadata',
            Common__pb2.Empty.SerializeToString,
            Image__pb2.ImageMetadata.FromString,
            options, channel_credentials,
            call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def get_metadatas(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/halodi.protobuf.sensors.ImageServer/get_metadatas',
            Common__pb2.Empty.SerializeToString,
            Image__pb2.ImagesMetadata.FromString,
            options, channel_credentials,
            call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def get_sensor_pose(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/halodi.protobuf.sensors.ImageServer/get_sensor_pose',
            Common__pb2.Empty.SerializeToString,
            Geometry__pb2.PoseWithMetadata.FromString,
            options, channel_credentials,
            call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def get_latest_timestamp(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_unary(request, target, '/halodi.protobuf.sensors.ImageServer/get_latest_timestamp',
            Common__pb2.Empty.SerializeToString,
            Common__pb2.Timestamp.FromString,
            options, channel_credentials,
            call_credentials, compression, wait_for_ready, timeout, metadata)

    @staticmethod
    def stream(request,
            target,
            options=(),
            channel_credentials=None,
            call_credentials=None,
            compression=None,
            wait_for_ready=None,
            timeout=None,
            metadata=None):
        return grpc.experimental.unary_stream(request, target, '/halodi.protobuf.sensors.ImageServer/stream',
            Common__pb2.Empty.SerializeToString,
            Common__pb2.DataChunk.FromString,
            options, channel_credentials,
            call_credentials, compression, wait_for_ready, timeout, metadata)
