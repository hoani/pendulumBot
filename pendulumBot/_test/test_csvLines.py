from pendulumBot.utilities import csvLines

class TestCsvLinesBasic:
  def test_initialization(self):
    log = csvLines.CsvLines()
    assert(log != None)

  def test_empty_header(self):
    log = csvLines.CsvLines()
    assert(log.header_string() == "\n")

  def test_empty_pop(self):
    log = csvLines.CsvLines()
    assert(log.pop_string() == "\n")

  def test_change_terminator(self):
    log = csvLines.CsvLines(terminator = ":")
    assert(log.pop_string() == ":")


class TestCsvLinesHeader:
  def setup_method(self):
    self.log = csvLines.CsvLines()

  def test_single_item(self):
    self.log['Item1'] = None
    assert(self.log.header_string() == "Item1\n")

  def test_multiple_items(self):
    self.log['Item1'] = None
    self.log['AnotherOne'] = None
    assert(self.log.header_string() == "Item1,AnotherOne\n")


class TestCsvLinesReady:
  def setup_method(self):
    self.log = csvLines.CsvLines()
    self.log['Item1'] = None
    self.log['Item2'] = None

  def test_is_not_ready(self):
    assert(self.log.ready() == False)

  def test_is_not_ready_yet(self):
    self.log['Item1'] = '1'
    self.log['Item2'] = None
    assert(self.log.ready() == False)

  def test_is_ready(self):
    self.log['Item1'] = '1'
    self.log['Item2'] = '2'
    assert(self.log.ready() == True)

class TestCsvLinesPop:
  def setup_method(self):
    self.log = csvLines.CsvLines()
    self.log['Item1'] = None
    self.log['Item2'] = None

  def test_not_ready(self):
    assert(self.log.pop_string() == "")

  def test_ready(self):
    self.log['Item1'] = '1'
    self.log['Item2'] = '2'
    assert(self.log.pop_string() == "1,2\n")

  def test_custom_separator(self):
    self.log = csvLines.CsvLines(separator="::FOO::")
    self.log['Item1'] = '1'
    self.log['Item2'] = '2'
    assert(self.log.header_string() == "Item1::FOO::Item2\n")
    assert(self.log.pop_string() == "1::FOO::2\n")

  def test_pop_consumes_all_lines(self):
    self.log = csvLines.CsvLines()
    self.log['Item1'] = '1'
    self.log['Item2'] = '2'
    _ = self.log.pop_string()
    assert(self.log.ready() == False)
    assert(self.log["Item1"] == None)
    assert(self.log["Item2"] == None)


class TestCsvLinesAdd:
  def setup_method(self):
    self.log = csvLines.CsvLines()

  def test_add_single_element(self):
    self.log.add("Test")
    assert("Test" in self.log.keys())

  def test_add_sets_none(self):
    self.log.add("Test")
    assert(self.log["Test"] == None)

class TestCsvLinesset:
  def setup_method(self):
    self.log = csvLines.CsvLines()
    self.log['Item1'] = None
    self.log['Item2'] = None
    self.log['Item3'] = None

  def test_set_single(self):
    self.log.set("Item1", 1)
    assert(self.log["Item1"] == 1)

  def test_set_multiple(self):
    self.log.set(["Item1", "Item2", "Item3"], [1, 2,3])
    assert(self.log["Item1"] == 1)
    assert(self.log["Item2"] == 2)
    assert(self.log["Item3"] == 3)

  def test_set_mismatched_lengths(self):
    self.log.set(["Item1", "Item2", "Item3"], [1])
    assert(self.log["Item1"] == 1)
    assert(self.log["Item2"] == None)
    assert(self.log["Item3"] == None)

  def test_set_unordered(self):
    self.log.set(["Item2", "Item1", "Item3"], [2, 1, 3])
    assert(self.log["Item1"] == 1)
    assert(self.log["Item2"] == 2)
    assert(self.log["Item3"] == 3)

  def test_set_invalid_item(self):
    self.log.set(["Item1", "Item2", "Item3", "Item4"], [1, 2, 3, 4])
    assert("Item4" not in self.log.keys())
