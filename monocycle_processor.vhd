--COMPONENTES DATAPATH
--Registrador de 64 bits
library IEEE;
use IEEE.numeric_bit.all;

entity reg64 is
  port (
    clock, reset: in bit;
    D: in  bit_vector(63 downto 0);
    Q: out bit_vector(63 downto 0)
  );
end entity;

architecture arch_reg64 of reg64 is
  signal dado: bit_vector(63 downto 0);
begin
  process(clock, reset)
  begin
    if reset = '1' then
      dado <= (others=>'0');
    elsif (clock'event and clock='1') then
        dado <= D;
    end if;
  end process;
  Q <= dado;
end architecture;



--REGFILE
library IEEE;
use IEEE.numeric_bit.all;
use IEEE.math_real.ceil;
use IEEE.math_real.log2;

entity regfile is
  generic(
    regn     : natural := 32;    --Numero de registers, de 0 a regn-1
    wordSize : natural := 64     --Tamanho da palavra de cada register
    );
  port(
    clock        : in  bit;
    reset        : in  bit;
    regWrite     : in  bit;
    rr1, rr2, wr : in bit_vector(natural(ceil(log2(real(regn))))-1 downto 0);
    d            : in bit_vector(wordSize-1 downto 0);
    q1, q2       : out bit_vector(wordSize-1 downto 0)
    );
end regfile;

architecture regfile_arch of regfile is
  type vetor is array (0 to regn-1) of bit_vector(wordSize-1 downto 0);
  signal registers : vetor;
  
begin
  ff: process(clock, reset)
  begin
    --Reset
    if reset = '1' then
      registers <= (others =>(others=>'0'));

    --Escrita  
    elsif clock = '1' and clock'event then
      if regWrite = '1'  then
        registers(to_integer(unsigned(wr))) <= d;
        registers(regn-1) <= (others=>'0'); --Zera o ultimo
      end if;
    end if;
  end process;
  
  --Leitura
  q1 <= registers(to_integer(unsigned(rr1)));
  q2 <= registers(to_integer(unsigned(rr2)));
  
end regfile_arch;

--FULL ADDER
library IEEE;
use IEEE.numeric_bit.all;

entity fulladder is
  port(
    a, b, cin : in bit;
    s, cout   : out bit
    );
end entity;

architecture structural of fulladder is
  signal axorb : bit;
begin
  axorb <= a xor b;
  s     <= axorb xor cin;
  cout  <= (axorb and cin) or (a and b);
end architecture;


--ULA 1BIT
library IEEE;
use IEEE.numeric_bit.all;

entity alu1bit is
  port(
    a, b, less, cin             : in bit;
    result, cout, set, overflow : out bit;
    ainvert, binvert            : in bit;
    operation                   : in bit_vector(1 downto 0)
    );
end entity;

architecture alu1bit_arch of alu1bit is
  signal signala, signalb, carryout : bit;
  signal andula, orula, addula      : bit;

  component fulladder
    port(
      a, b, cin : in bit;
      s, cout   : out bit
      );
  end component;
  
begin
  adder : fulladder
    port map(signala, signalb, cin, addula, carryout);

  cout <= carryout;

  with ainvert select
    signala <= a      when '0',
               not(a) when '1';

  with binvert select
    signalb <= b      when '0',
               not(b) when '1';

  andula <= signala and signalb;
  orula  <= signala or signalb;

  with operation select
    result <= andula when "00",
              orula  when "01",
              addula when "10",
              b      when "11";

  overflow <= cin xor carryout;
  
  set <= addula;
  
end alu1bit_arch;

--ULA COMPLETA
library IEEE;
use IEEE.numeric_bit.all;

entity alu is
  generic(
    size : natural := 10
    );
  port(
    A, B : in  bit_vector(size-1 downto 0); --inputs
    F    : out bit_vector(size-1 downto 0); --output
    S    : in  bit_vector(3 downto 0);      --selecao de op
    Z    : out bit;                         --flag de zero 
    Ov   : out bit;                         --flag de overflow
    Co   : out bit                          --carry out
    );
end entity alu;

architecture alu_arch of alu is
  signal result, cout, set, overflowA, detectaZero : bit_vector(size-1 downto 0);
  signal operacao                           : bit_vector(1 downto 0);
  signal inverteA, inverteB                 : bit;

  component alu1bit is
    port(
    a, b, less, cin             : in bit;
    result, cout, set, overflow : out bit;
    ainvert, binvert            : in bit;
    operation                   : in bit_vector(1 downto 0)
    );
  end component;
  
begin
  
  alus : for i in size-2 downto 1 generate
    aluss: alu1bit
      port map(A(i), B(i), '0', cout(i-1), result(i), cout(i), set(i), overflowA(i), inverteA, inverteB, operacao);
  end generate;

  
  primeiro: alu1bit
    port map(A(0), B(0), set(size-1), inverteB, result(0), cout(0), set(0), overflowA(0), inverteA, inverteB, operacao);

  ultimo: alu1bit
    port map(A(size-1), B(size-1), '0', cout(size-2), result(size-1), cout(size-1), set(size-1), overflowA(size-1), inverteA, inverteB, operacao);

  
  inverteA <= S(3);
  inverteB <= S(2);
  operacao <= S(1) & S(0);
  
  F <= result;
  
  --Carry out
  Co <= cout(size-1);

  --Verificacao de overflow
  Ov <= cout(size-2) xor cout(size-1);
  
  --Deteccao de zero
  detectaZero <= (0 => '0', others => '0');
  Z <= '1' when (unsigned(result) = unsigned(detectaZero)) else '0';
  
end architecture;

--Comparador de 11 bits
library IEEE;
use IEEE.numeric_bit.all;

entity comp11 is
  port(inpA, inpB: in bit_vector(10 downto 0);
       equal     : out bit
  );
end entity;

architecture arch_comp11 of comp11 is
  signal bitA, bitB: unsigned(10 downto 0);
begin
  bitA  <= unsigned(inpA);
  bitB  <= unsigned(inpB);
  equal <= '1' when (bitA = bitB) else '0';
end architecture;

--Comparador de 8 bits
library IEEE;
use IEEE.numeric_bit.all;

entity comp8 is
  port(inpA, inpB: in bit_vector(7 downto 0);
       equal     : out bit
  );
end entity;

architecture arch_comp8 of comp8 is
  signal bitA, bitB: unsigned(7 downto 0);
begin
  bitA  <= unsigned(inpA);
  bitB  <= unsigned(inpB);
  equal <= '1' when (bitA = bitB) else '0';
end architecture;

--Comparador de 6 bits
library IEEE;
use IEEE.numeric_bit.all;

entity comp6 is
  port(inpA, inpB: in bit_vector(5 downto 0);
       equal     : out bit
  );
end entity;

architecture arch_comp6 of comp6 is
  signal bitA, bitB: unsigned(5 downto 0);
begin
  bitA  <= unsigned(inpA);
  bitB  <= unsigned(inpB);
  equal <= '1' when (bitA = bitB) else '0';
end architecture;


--SIGNEXTEND
library IEEE;
use IEEE.numeric_bit.all;

entity signExtend is
  port(
    i: in  bit_vector(31 downto 0);
    o: out bit_vector(63 downto 0)
    );
end signExtend;

architecture signArch of signExtend is
  signal opcodeD1, opcodeD2: bit_vector(10 downto 0);
  signal opcodeCBZ         : bit_vector(7 downto 0);
  signal opcodeB           : bit_vector(5 downto 0);

  signal igualD1, igualD2, igualD, igualCBZ, igualB: bit;
  
  signal formatoD, formatoCBZ, formatoB, zero: bit_vector(63 downto 0);

  signal comparacao: bit_vector(2 downto 0);
  
  component comp11 is
    port(
      inpA, inpB: in bit_vector(10 downto 0);
      equal     : out bit
      );
  end component;

  component comp8 is
    port(
      inpA, inpB: in bit_vector(7 downto 0);
      equal     : out bit
      );
  end component;

  component comp6 is
    port(
      inpA, inpB: in bit_vector(5 downto 0);
      equal     : out bit
      );
  end component;
  
  
begin
  opcodeD1  <= "11111000010";
  opcodeD2  <= "11111000000";
  opcodeCBZ <= "10110100";
  opcodeB   <= "000101";
  
  c111: comp11
    port map(opcodeD1, i(31 downto 21), igualD1);
  c112: comp11
    port map(opcodeD2, i(31 downto 21), igualD2);
  c8: comp8
    port map(opcodeCBZ, i(31 downto 24), igualCBZ);
  c6: comp6
    port map(opcodeB, i(31 downto 26), igualB);

  igualD <= igualD1 or igualD2;

  comparacao <= igualD & igualCBZ & igualB;

  with i(20) select
    formatoD <= (63 downto 9 => '0') & i(20 downto 12) when '0',
                (63 downto 9 => '1') & i(20 downto 12) when '1';

  with i(23) select
    formatoCBZ <= (63 downto 19 => '0') & i(23 downto 5) when '0',
                  (63 downto 19 => '1') & i(23 downto 5) when '1';

  with i(25) select
    formatoB <= (63 downto 26 => '0') & i(25 downto 0) when '0',
                (63 downto 26 => '1') & i(25 downto 0) when '1';

  zero <= (0 => '0', others => '0');
  
  with comparacao select
    o <= formatoD   when "100",
         formatoCBZ when "010",
         formatoB   when "001",
         zero when others;

end architecture;

--SHIFTLEFT2
library IEEE;
use IEEE.numeric_bit.all;

entity Shiftleft2 is
  port(
    i1 : in  bit_vector(63 downto 0);
    o1 : out bit_vector(63 downto 0)
    );
end entity;

architecture shift_arch of Shiftleft2 is
begin
  o1 <= i1(61 downto 0) & "00";
end architecture;



--DATAPATH
library IEEE;
use IEEE.numeric_bit.all;
use IEEE.math_real.ceil;
use IEEE.math_real.log2;

entity datapath is
  port(
    --Common
    clock : in bit;
    reset : in bit;
    
    --From Control Unit
    reg2loc  : in bit;
    pcsrc    : in bit;
    memToReg : in bit;
    aluCtrl  : in bit_vector(3 downto 0);
    aluSrc   : in bit;
    regWrite : in bit;
    
    --To Control Unit
    opcode : out bit_vector(10 downto 0);
    zero   : out bit;

    --IM Interface
    imAddr : out bit_vector(63 downto 0);
    imOut  : in  bit_vector(31 downto 0);

    --DM Interface
    dmAddr : out bit_vector(63 downto 0);
    dmIn   : out bit_vector(63 downto 0);
    dmOut  : in  bit_vector(63 downto 0)
    );
end entity datapath;

architecture datapath_arch of datapath is
  --signals do banco de registers
  signal readReg2 : bit_vector(4 downto 0);
  signal writeData, readData1, readData2 : bit_vector(63 downto 0);

  --PC
  signal entradaPC, saidaPC : bit_vector(63 downto 0);

  --shift left 2
  signal entradaShift, saidaShift : bit_vector(63 downto 0);

  --MUXes
  signal ms1, me30, me40, me41 : bit_vector(63 downto 0);

  component reg64
  port (
    clock, reset: in bit;
    D: in  bit_vector(63 downto 0);
    Q: out bit_vector(63 downto 0)
    );
  end component;
  
  component regfile
  generic(
    regn     : natural := 32;    --Numero de registers, de 0 a regn-1
    wordSize : natural := 64     --Tamanho da palavra de cada register
    );
  port(
    clock        : in  bit;
    reset        : in  bit;
    regWrite     : in  bit;
    rr1, rr2, wr : in  bit_vector(natural(ceil(log2(real(regn))))-1 downto 0);
    d            : in  bit_vector(wordSize-1 downto 0);
    q1, q2       : out bit_vector(wordSize-1 downto 0)
    );
  end component;

  component alu
  generic(
    size : natural := 10
    );
  port(
    A, B : in  bit_vector(size-1 downto 0); --inputs
    F    : out bit_vector(size-1 downto 0); --output
    S    : in  bit_vector(3 downto 0);      --selecao de op
    Z    : out bit;                         --flag de zero 
    Ov   : out bit;                         --flag de overflow
    Co   : out bit                          --carry out
    );
  end component;

  component signExtend
  port(
    i: in  bit_vector(31 downto 0);
    o: out bit_vector(63 downto 0)
    );
  end component;

  component Shiftleft2
  port(
    i1 : in  bit_vector(63 downto 0);
    o1 : out bit_vector(63 downto 0)
    );
  end component;

begin
  PC0: reg64
    port map(
      clock => clock,
      reset => reset,
      D     => entradaPC,
      Q     => saidaPC
      );
  
  registers: regfile
    generic map(
      regn     => 32,
      wordSize => 64
      )
    port map(
      clock    => clock,
      reset    => reset,
      regWrite => regWrite,
      rr1      => imOut(9 downto 5),
      rr2      => readReg2,
      wr       => imOut(4 downto 0),
      d        => writeData,
      q1       => readData1,
      q2       => readData2
      );

  ULA1: alu
    generic map(
      size => 64
      )
    port map(
      A  => readData1,
      B  => ms1,
      F  => me30,
      S  => aluCtrl,
      Z  => zero,
      Ov => open,
      Co => open
      );

  ULA2: alu
    generic map(
      size => 64
      )
    port map(
      A  => saidaPC,
      B  => "0000000000000000000000000000000000000000000000000000000000000100",
      F  => me40,
      S  => "0010",
      Z  => open,
      Ov => open,
      Co => open
      );

  ULA3: alu
    generic map(
      size => 64
      )
    port map(
      A  => saidaPC,
      B  => saidaShift,
      F  => me41,
      S  => "0010",
      Z  => open,
      Ov => open,
      Co => open
      );

  sign: signExtend
    port map(
      i => imOut,
      o => entradaShift
      );

  shifter: Shiftleft2
    port map(
      i1 => entradaShift,
      o1 => saidaShift
      );

  --MUXes
  with aluSrc select
    ms1 <= readData2    when '0',
           entradaShift when '1';

  with reg2loc select
    readReg2 <= imOut(20 downto 16) when '0',
                imOut(4 downto 0)   when '1';

  with memToReg select
    writeData <= dmOut when '1',
                 me30  when '0';

  with pcsrc select
    entradaPC <= me40 when '0',
          me41 when '1';

  dmAddr <= me30;
  dmIn <= readData2;

  imAddr <= saidaPC;

  opcode <= imOut(31 downto 21);

end architecture;


--ALU CONTROL
library IEEE;
use IEEE.numeric_bit.all;

entity alucontrol is
  port(
    aluop  : in  bit_vector(1 downto 0);
    opcode : in  bit_vector(10 downto 0);
    aluCtrl: out bit_vector(3 downto 0)
    );
end entity;

architecture alucontrolArch of alucontrol is
  signal saidaR, comparacao: bit_vector(3 downto 0);
  signal igualADD, igualSUB, igualAND, igualOR: bit;

  component comp11 is
    port(
      inpA, inpB: in bit_vector(10 downto 0);
      equal     : out bit
      );
  end component;
  
  
begin
  cADD: comp11 port map(opcode, "10001011000", igualADD);
  cSUB: comp11 port map(opcode, "11001011000", igualSUB);
  cAND: comp11 port map(opcode, "10001010000", igualAND);
  cOR : comp11 port map(opcode, "10101010000", igualOR );

  comparacao <= igualADD & igualSUB & igualAND & igualOR;
  
  with aluop select
    aluCtrl <= "0010" when "00",
               "0111" when "01",
               saidaR when "10",
               "1111" when "11";

  with comparacao select
    saidaR <= "0010" when "1000",
              "0110" when "0100",
              "0000" when "0010",
              "0001" when "0001",
              "1111" when others;

end architecture;


--UNIDADE DE CONTROLE
library IEEE;
use IEEE.numeric_bit.all;

entity controlunit is
  port(
    --to datapath
    reg2loc      : out bit;
    uncondBranch : out bit;
    branch       : out bit;
    memRead      : out bit;
    memToReg     : out bit;
    aluOp        : out bit_vector(1 downto 0);
    memWrite     : out bit;
    aluSrc       : out bit;
    regWrite     : out bit;
    --from datapath
    opcode       : in  bit_vector(10 downto 0)
    );
end entity;

architecture controlunitArch of controlunit is
  signal igualLDUR, igualSTUR, igualADD, igualSUB    : bit;
  signal igualAND, igualORR, igualCBZ, igualB, igualR: bit;
  signal comparacao: bit_vector(4 downto 0);

  component comp11 is
    port(
      inpA, inpB: in bit_vector(10 downto 0);
      equal     : out bit
      );
  end component;

  component comp8 is
    port(
      inpA, inpB: in bit_vector(7 downto 0);
      equal     : out bit
      );
  end component;

  component comp6 is
    port(
      inpA, inpB: in bit_vector(5 downto 0);
      equal     : out bit
      );
  end component;
  
begin
  cLDUR: comp11 port map(opcode, "11111000010", igualLDUR);
  cSTUR: comp11 port map(opcode, "11111000000", igualSTUR);
  cADD : comp11 port map(opcode, "10001011000", igualADD );
  cSUB : comp11 port map(opcode, "11001011000", igualSUB );
  cAND : comp11 port map(opcode, "10001010000", igualAND );
  cORR : comp11 port map(opcode, "10101010000", igualORR );
  cCBZ : comp8  port map(opcode(10 downto 3), "10110100", igualCBZ);
  cB   : comp6  port map(opcode(10 downto 5), "000101", igualB);

  igualR <= igualADD or igualSUB or igualAND or igualORR;

  comparacao <= igualLDUR & igualSTUR & igualCBZ & igualB & igualR;
            
  reg2loc      <= igualSTUR or igualCBZ or igualB;
  uncondBranch <= igualB;
  branch       <= igualCBZ;
  memRead      <= igualLDUR;
  memToReg     <= igualLDUR;   --memToReg <= igualLDUR or igualB;(?)
  memWrite     <= igualSTUR;
  aluSrc       <= igualLDUR or igualSTUR;
  regWrite     <= igualLDUR or igualR;

  with comparacao select
    aluOp <= "01" when "00100",
             "11" when "00010",
             "10" when "00001",
             "00" when others;

end architecture;

--POLILEGSC
library IEEE;
use IEEE.numeric_bit.all;

entity polilegsc is
  port(
    clock, reset : in bit;

   --DATA MEMORY
    dmem_addr : out bit_vector(63 downto 0);
    dmem_dati : out bit_vector(63 downto 0);
    dmem_dato : in  bit_vector(63 downto 0);
    dmem_we   : out bit;

   --INSTRUCTION MEMORY
    imem_addr : out bit_vector(63 downto 0);
    imem_data : in  bit_vector(31 downto 0)
    );
end entity;

architecture polileg_arch of polilegsc is
  --signals ALU Control
  signal aluOpp   : bit_vector(1 downto 0);
  signal opcodep  : bit_vector(10 downto 0);
  signal aluCtrlp : bit_vector(3 downto 0);

  --signals controlunit
  signal reg2locp, uncondBranchp, branchp, memReadp, memToRegp, memWritep, aluSrcp, regWritep : bit;

  --signals datapath
  signal pcsrcp, zerop, bz : bit;

  component datapath is
  port(
    --Common
    clock : in bit;
    reset : in bit;
    
    --From Control Unit
    reg2loc  : in bit;
    pcsrc    : in bit;
    memToReg : in bit;
    aluCtrl  : in bit_vector(3 downto 0);
    aluSrc   : in bit;
    regWrite : in bit;
    
    --To Control Unit
    opcode : out bit_vector(10 downto 0);
    zero   : out bit;

    --IM Interface
    imAddr : out bit_vector(63 downto 0);
    imOut  : in  bit_vector(31 downto 0);

    --DM Interface
    dmAddr : out bit_vector(63 downto 0);
    dmIn   : out bit_vector(63 downto 0);
    dmOut  : in  bit_vector(63 downto 0)
    );
  end component;

  component alucontrol is
  port(
    aluop  : in  bit_vector(1 downto 0);
    opcode : in  bit_vector(10 downto 0);
    aluCtrl: out bit_vector(3 downto 0)
    );
  end component;

  component controlunit is
  port(
    reg2loc      : out bit;
    uncondBranch : out bit;
    branch       : out bit;
    memRead      : out bit;
    memToReg     : out bit;
    aluOp        : out bit_vector(1 downto 0);
    memWrite     : out bit;
    aluSrc       : out bit;
    regWrite     : out bit;
    opcode       : in  bit_vector(10 downto 0)
    );
  end component;

begin
  dp: datapath
    port map(
      clock    => clock,
      reset    => reset,
      reg2loc  => reg2locp,
      pcsrc    => pcsrcp,
      memToReg => memToRegp,
      aluCtrl  => aluCtrlp,
      aluSrc   => aluSrcp,
      regWrite => regWritep,
      opcode   => opcodep,
      zero     => zerop,
      imAddr   => imem_addr,
      imOut    => imem_data,
      dmAddr   => dmem_addr,
      dmIn     => dmem_dati,
      dmOut    => dmem_dato
      );

  aluctrl0: alucontrol
    port map(
      aluop   => aluOpp,
      opcode  => opcodep,
      aluCtrl => aluCtrlp
      );

  ctrlunit0: controlunit
    port map(
      reg2loc      => reg2locp,
      uncondBranch => uncondBranchp,
      branch       => branchp,
      memRead      => memReadp,
      memToReg     => memToRegp,
      aluOp        => aluOpp,
      memWrite     => memWritep,
      aluSrc       => aluSrcp,
      regWrite     => regWritep,
      opcode       => opcodep
      );

  bz     <= branchp and zerop;
  pcsrcp <= uncondBranchp or bz;

  dmem_we <= memWritep;

end architecture;
