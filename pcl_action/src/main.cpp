#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include <std_msgs/Int8.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <chrono>
#include <pcl/filters/passthrough.h> 
#include "pcl_action/custom.h"

/*! \brief Contiene le costatni matematiche.
 */
#define _USE_MATH_DEFINES

/*! \brief Rappresenta la classe di base della libreria PCL e puo' contenere una collezione di punti 3D.
 */
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//! Costante che rappresenta l'altezza del piano di lavoro su cui vengono svolte le operazioni.
const float CZ = 1.02; 

/** Variabile che contiene l'informazione sulla tipologia di blocco individuato e puo' avere i seguenti valori: 
* 0 -> Se il blocco e' un cubo blu.
* 1 -> Se il blocco e' un cubo giallo.
* 2 -> Se il blocco e' un cubo rosso.
* 3 -> Se il blocco e' un parallelepipedo.
*/
int id;

//! Variabile che contiene il valore di rotazione del blocco individuato.
float alpha=0;

//! Variabile che contiene il valore della coordinata x del centro del blocco individuato.
float centrox=0;

//! Variabile che contiene il valore della coordinata y del centro del blocco individuato.
float centroy=0;


/*! \brief Funzione getbbs di tipo void che legge dal nodo "darknet_ros/bounding_boxes" l'ID del blocco classificato e lo salva nella variabile id.
* @param msg : variabile dove sono memorizzati i dati pubblicati dal nodo "darknet_ros/bounding_boxes".
*/
void getbbs(const darknet_ros_msgs::BoundingBoxes::ConstPtr &msg){

  //Nella variabile id vine salvato l'ID del blocco classificato da YOLO
  id = msg->bounding_boxes[0].id; 
}



/*! \brief Funzione getcloud di tipo void che esegue le operazioni di filtraggio, segmentazione, clustering, sulla nuvola di punti ricavata dal nodo "zed2/zed_node/point_cloud/cloud_registered".
* Inoltre, eseguiti tutti i passagi appena elencati, ricava i vertici dei blocchi estratti in modo da poter ricavarne le coordinate del centro e il suo angolo di rotazione.
* @param pCloud : variabile di tipo pointcloud2 dove sono memorizzati i dati pubblicati dal nodo "zed2/zed_node/point_cloud/cloud_registered".
*/
void getcloud(const sensor_msgs::PointCloud2ConstPtr& pCloud){

  auto started = std::chrono::high_resolution_clock::now(); //Inizio conteggio per calcolo tempo esecuzione programma

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  // Viene salvata la nuvola di punti ricavata dal nodo "zed2/zed_node/point_cloud/cloud_registered".
  pcl::fromROSMsg(*pCloud, *cloud); 

  pcl::PCDWriter writer;

  std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl;

  // Vengono applicati i filtri lungo i tre assi cartesiani, mantenendo solo i punti con z < 40cm, 0cm < x < 40cm e -20cm < y < 20cm.
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 0.40);
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("y");
  pass.setFilterLimits (-0.2, 0.2);
  pass.filter (*cloud);

  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0, 0.4); 
  pass.filter (*cloud);

  std::cout << "PointCloud representing the Cluster: " << cloud->size () << " data points." << std::endl;
  
  
  // Viene effettuato il downsampling della nuvola di punti per alleggerirlae e rendere piu' rapide le azioni sucessive.
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.0005f, 0.0005f, 0.0005f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*


  // Viene effettuata la segmentazione per individuare i piani presenti nella nuvola di punti tenendo in cosiderazione solo quelli composdi da un numero di punti superiore al 30% di quelli totali.
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);
  int nr_points = (int) cloud_filtered->size ();
  while (cloud_filtered->size () > 0.3 * nr_points)
  {
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

    // Vengono rimossi i piani individuati nella nuvola di punti. 
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  // Viene analizzata la nuvola di punti e suddivisa in gruppi di punti appartenenti ciascuno ad un oggetto, raggruppando i punti vicini tra loro meno di 1cm e salvando solo i gruppi conteneti 20 punti.
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.01); 
  ec.setMinClusterSize (200);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  int j = 0;

  // Ciclo che esegue le operazioni di calcolo del centro e angolo per ogni blocco individuato
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& idx : it->indices)
    cloud_cluster->push_back ((*cloud_filtered)[idx]); //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
 
    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;

    // Per ogni blocco vengono individuati i suoi vertici.  
    float ax=100, bx=100, cx=-100, dx=-100, ay=100, by=-100, cy=-100, dy=100; 

    for (const auto& point: *cloud_cluster){ // Vertice A
      if(point.z < ay){
          ay=point.z;
          ax=point.y;
      }
    };

    for (const auto& point: *cloud_cluster){ // Vertice B
      if( point.y < bx){   
          by=point.z;
          bx=point.y;
      }
    };

    for (const auto& point: *cloud_cluster){ // Vertice C
      if(point.z > cy){
        cy=point.z;
        cx=point.y;
      }
    };

    for (const auto& point: *cloud_cluster){ // Vertice D
      if(point.y > dx){
          dy=point.z;
          dx=point.y;
      }
    };


    std::cerr << "Vetice A: " << ax << ", " << ay << std::endl;
    std::cerr << "Vetice B: " << bx << ", " << by << std::endl;
    std::cerr << "Vetice C: " << cx << ", " << cy << std::endl;
    std::cerr << "Vetice D: " << dx << ", " << dy << std::endl;


    float ab=0, ad=0, x2=0, y2=0;

    ab = sqrt((bx-ax)*(bx-ax)+(by-ay)*(by-ay));
    ad = sqrt((dx-ax)*(dx-ax)+(dy-ay)*(dy-ay));

    if(ab>ad){
      x2=bx;
      y2=by;
      std::cerr << "Lato lungo AB di lunghezza: " << ab << std::endl;
    }else{ 
      x2=dx;
      y2=dy;
      std::cerr << "Lato lungo AD di lunghezza: " << ad << std::endl; 
    }

    // Viene calcolato l'angolo rispetto all'asse x e viene salvato nella variabile alpha.
    float m=0, num=0, den=0;

    num = y2-ay;
    den = x2-ax;

    m = num/den;
    alpha = atan(m)*(180/M_PI); 
    alpha=alpha*(-1);

    std::cerr << "Angolo alpha: " << alpha << std::endl;

    // Viene calcolato il centro del blocco, e le coordinate vengono salvate nelle variabili centrox e centroy
    float m1=0, m2=0, q1=0, q2=0;

    m1=(cy-ay)/(cx-ax); 
    m2=(dy-by)/(dx-bx);
    q1=(cx*ay-ax*cy)/(cx-ax);
    q2=(dx*by-bx*dy)/(dx-bx);

    centrox=(q2-q1)/(m1-m2); 
    centroy=centrox*m1+q1; 
    centrox=centrox*(-1);

    std::cerr << "Coordinate centro: " << centrox << ", " << centroy << std::endl;

  };

  auto done = std::chrono::high_resolution_clock::now(); 
  std::cout <<"Execution time: "<< std::chrono::duration_cast<std::chrono::milliseconds>(done-started).count()<<" ms"<<std::endl; 

}


/*! \brief Nel main di questo programma vengono eseguite le sottoscrizioni e le pubblicazioni ai diversi nodi utilizzati.
* Inoltre, attraverso un ciclo while, e' presente un semplice menu' per interagire con l'utente. 
*/
int main(int argc, char **argv){
 
  ros::init(argc, argv, "action");
  ros::NodeHandle n;
  ros::Rate rate(10); 

  char choice;
  bool v;

  // Sottoscrizione hai nodi necessari per ricevere le informazioni
  ros::Subscriber yolo = n.subscribe("darknet_ros/bounding_boxes", 1, getbbs);
  ros::Subscriber cloudpoint = n.subscribe("zed2/zed_node/point_cloud/cloud_registered", 1, getcloud); 

  //Creazione nodo di pubblicazione delle informazioni prodotte
  ros::Publisher pub = n.advertise<pcl_action::custom>("custom_msg", 10);

    // Qui e' presente un ciclo while che permette un'interazione con l'utenete, il quale puo decidere se analizzare un altro blocco o chiudere il programma.
    while (ros::ok() && choice != 'q' && choice != 'Q'){

    pcl_action::custom msg;
        
    std::cout << "------------------------------------------------------------------" << std::endl;
    std::cout << "Posiziona un blocco sotto la telecamera e premi un taso per continuare,\nq per uscire" << std::endl;
		std::cin >> choice;
    std::cout << "------------------------------------------------------------------" << std::endl;
        
    if(choice != 'q' && choice != 'Q'){

      // Vengono ascoltati i messaggi ai nodi sottoscritti.             
      ros::spinOnce();

      // Viene verificato se il blocco identificato e' un cubo. 
      if(id == 0 || id == 1 || id == 2 || id == 3){
        v = true; 
      }else{
        v = false;
      };

      // Viene creato il messaggio, inserendo correttamente ogni variabile necessaria.
      msg.cx = centrox; 
      msg.cy = centroy; 
      msg.cz = CZ;
      msg.r = -3.1415927;
      msg.p = 0.0;
      msg.y = alpha;
      msg.cube = true;
      msg.id = id;

      // Viene pubblicato il messaggio attraverso il publisher pub.
      pub.publish(msg);
          
    };
        
  }
    
return 0;}