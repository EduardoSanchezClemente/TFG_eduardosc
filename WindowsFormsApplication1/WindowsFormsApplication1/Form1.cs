using System;
using System.Threading;
using System.Windows.Forms;
using System.IO;


namespace WindowsFormsApplication1
{
    public partial class Form1 : Form
    {
        int var_rssi = -100;
        UInt16 var_node = 0;  //Como lo inicializo?
        UInt16 var_placa = 0;  //Como lo inicializo?
        UInt16 aux_nodo = 0;  //Como lo inicializo?
        UInt16 aux_placa = 0;
        int aux = 0;
        int aux2 = 0;
        int posicion_nodo = 0;
        int posicion_placa = 0;
        int [,] placa_nod = new int[10,10] { { -100, -100, -100, -100, -100, -100, -100, -100, -100, -100 }, { -100, -100, -100, -100, -100, -100, -100, -100, -100, -100 }, { -100, -100, -100, -100, -100, -100, -100, -100, -100, -100 }, { -100, -100, -100, -100, -100, -100, -100, -100, -100, -100 }, { -100, -100, -100, -100, -100, -100, -100, -100, -100, -100 }, { -100, -100, -100, -100, -100, -100, -100, -100, -100, -100 }, { -100, -100, -100, -100, -100, -100, -100, -100, -100, -100 }, { -100, -100, -100, -100, -100, -100, -100, -100, -100, -100 }, { -100, -100, -100, -100, -100, -100, -100, -100, -100, -100 }, { -100, -100, -100, -100, -100, -100, -100, -100, -100, -100 }, };
        //  int[][] placa_nod = new int[10][] { { -100, -100, -100, -100, -100, -100, -100, -100, -100, -100 }, new int[10], new int[10], new int[10], new int[10], new int[10], new int[10], new int[10], new int[10], new int[10] };
        int[] placa_RSSI = new int[10];

        UInt16[] placas = new UInt16[10];
        UInt16[] nodos = new UInt16[10];


        System.IO.FileStream outputFS;
        System.IO.StreamWriter outputWriter;

        String LecturasRSSI;    //Nombre del archivo donde se guardan los datos.
                                //C:\Users\eduardosc\Desktop\Prueba.txt

        int contador_datos = 0;

        bool started = false;
        Thread readThread;
        public Form1()
        {
            InitializeComponent();
        }

        private void start_button_Click(object sender, EventArgs e)
        {
            if (started)
            {
                start_button.Text = "START";
                started = false;



                try
                {
                    this.BeginInvoke((MethodInvoker)delegate         // runs on UI thread
                    {
                        trace_box.AppendText("***Finaliza el programa\r\n");
                    });
                }


                catch (TimeoutException) { }

            }
            else
            {
                started = true;
                start_button.Text = "STOP";
                this.BeginInvoke((MethodInvoker)delegate         // runs on UI thread
                {
                    trace_box.AppendText("***Comienza el programa\r\n");
                });
            
                readThread = new Thread(Read);
                if (serialPort1.IsOpen)
                {
                    serialPort1.Close();
                }
                this.serialPort1.PortName = text_serial_port.Text;
                serialPort1.Open();
                readThread.Start();

            }
        }

        public void Read()
        {

            try
            {
                byte[] data = new byte[1024];
                while (true)
                {
                    //Read preambule
                    int readed = serialPort1.Read(data, 0, 1);
                    Byte preamb = data[0];
                    while (readed == 0 || preamb != 0xAA)
                    {
                        readed = serialPort1.Read(data, 0, 1);
                        preamb = data[0];

                    }
                    string measure;
                    contador_datos++;

                    readed = serialPort1.Read(data, 0, 10);
                    UInt16 src_addr = BitConverter.ToUInt16(data, 0);
                    UInt16 node_addr = BitConverter.ToUInt16(data, 2);
                    int rssi = BitConverter.ToInt16(data, 3);

                    if (data[4] == 45)
                    {
                        rssi = ((data[5] - 48) * 10 + (data[6] - 48)) * -1;
                    }
                    else
                    {
                        rssi = ((data[5] - 48) * 10 + (data[6] - 48));
                    }

                    this.BeginInvoke((MethodInvoker)delegate         // runs on UI thread
                    {
                        trace_box.AppendText("La placa " + src_addr + " ha sido detectado por el nodo " + node_addr + " con un RSSI de " + rssi + "dBm" + "\r\n");
                    });

                    LecturasRSSI = "Lecturas.txt";

                    //a veces la interfaz se queda pillada con lo del outputWriter
                    outputFS = (System.IO.FileStream)File.Open(LecturasRSSI, FileMode.Append, FileAccess.Write);

                    outputWriter = new System.IO.StreamWriter(outputFS);
                    outputWriter.NewLine = "\r\n";

                    measure = src_addr + " " + node_addr + " " + rssi; 
                    outputWriter.WriteLine(measure);

                    outputWriter.Close();  //Cerramos el texto
                    outputFS.Close();




                    aux_nodo = node_addr;
                    aux_placa = src_addr;
                  
                    for (int i = 0; i < placas.Length - 1; i++)
                    {
                        if (placas[i] == aux_placa)
                        {
                            aux2 = 1;
                            if (placa_RSSI[i] <= rssi)
                            {
                                placa_RSSI[i] = rssi;
                                var_node = node_addr;
                                nodos[i] = node_addr;
                                var_placa = src_addr;
                                var_rssi = placa_RSSI[i];

                            }
                                                      
                        }
                    }
                    if (aux2 != 1)
                    {
                        nodos[posicion_placa] = aux_nodo;
                        var_rssi = rssi;
                        placas[posicion_placa] = aux_placa;
                        placa_RSSI[posicion_placa] = rssi;
                        posicion_placa++;
                    }

                  
                    aux = 0;
                    aux2 = 0;


                    this.BeginInvoke((MethodInvoker)delegate         // runs on UI thread
                    {
                        for (int i = 0; i < placas.Length - 1; i++)
                        {
                            if (placas[i] != 0)
                            {
                                trace_box.AppendText("El RSSI más grande de la placa " + placas[i] + " detectado por el nodo " + nodos[i] + " es de " + placa_RSSI[i] +" dBm" + "\r\n");
                            }
                        }
                                  
                    });



                }
            }
            catch (TimeoutException) { }

        }

        private void timer1_Tick_1(object sender, EventArgs e)
        {
            this.BeginInvoke((MethodInvoker)delegate         // runs on UI thread
            {
                trace_box.AppendText("Han pasado 20 segundos, reiniciamos valores\r\n");
                posicion_nodo = 0;
                posicion_placa = 0;
                var_node = 0;
                var_placa = 0;
                var_rssi = -100;
                for (int i = 0; i < nodos.Length - 1; i++)
                {
                    placas[i] = 0;
                    nodos[i] = 0;
                    placa_RSSI[i] = -100; 
                }
            });
        }
    }
}
