"""CompressedImage.format strings: image_transport must be able to decode them, and the
sequence number must come back out."""

from stretch_python_bridge import compressed_format_with_sequence, parse_compressed_format


def test_bare_codec_gets_image_transport_form():
    # image_transport reads the text before the first ";" as the pixel encoding.
    fmt = compressed_format_with_sequence("jpeg", 1234)
    assert fmt == "bgr8; jpeg compressed bgr8; seq=1234"
    assert fmt.split(";")[0] == "bgr8"
    assert parse_compressed_format(fmt) == ("bgr8; jpeg compressed bgr8", 1234)


def test_image_transport_formats_are_kept():
    fmt = compressed_format_with_sequence("16UC1; compressedDepth png", 7)
    assert fmt == "16UC1; compressedDepth png; seq=7"
    assert parse_compressed_format(fmt) == ("16UC1; compressedDepth png", 7)


def test_no_sequence_number_leaves_it_alone():
    assert compressed_format_with_sequence("jpeg", None) == "jpeg"
